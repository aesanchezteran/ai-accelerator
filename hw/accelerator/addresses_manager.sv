/* 
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025 Quito - Ecuador  

Module Details:
-------------------------------------------------------------------------------
Module Name      : addresses_manager
Project          : Dragon AI Accelerator
Description      : 
  The `addresses_manager` module is responsible for managing and generating 
  memory addresses for different operations in the Dragon AI Accelerator. 
  This includes managing addresses for activation storage and retreival from 
  the scratchpad, partial sum operations, and matrix multiplication computation.
  
  It provides to the Dragon AI Accelerator the desired addresses that are 
  comming from the processor instruction bus. Additionally, it decodes both row
  IDs and addresses from the instruction, by partitioning the ISA according to
  its ISA. Finally, it ensure synchronization between different parts of the 
  accelerator.
  
Inputs:
  - clk                                 : Clock signal.
  - rst                                 : Reset signal.
  - enable                              : Enables the module's operation.
  - opcode                              : The opcode for the current operation.
  - instr_pacc_address_ps_1             : Address for the first partial sum element.
  - instr_pacc_address_ps_2             : Address for the second partial sum element.
  - instr_mm_end_address_activation     : End address for the activation memory.
  - instr_mm_start_address_activation   : Start address for the activation memory.
  - instr_mm_address_ps                 : Address for matrix multiplication partial sum storage in scratchpad.
  - instr_ws_row_id                     : Row ID for weight storage.
  - instr_as_address_act                : Address for activation storage.
  - instr_ps_address_ps                 : Address for partial sum storage.
  - instr_pc_address_ps                 : Address for partial sum collect.

Outputs:
  - address_act             : Address for activation storage.
  - address_ps_a            : Address for partial sum scratchpad A.
  - address_ps_b            : Address for partial sum scratchpad B.
  - row_id                  : Row ID for the operation.
  - mm_act_done             : Indicates if matrix multiplication is complete.
  - ps_we                   : Write enable signal for partial sum.
  - op_done                 : Indicates if the current operation is complete.
  - mm_pe_selector          : Selector for matrix multiplication PE.
*/
`include "enums.sv"

module addresses_manager#(
    // Parameters Override  
	parameter integer ARRAY_HEIGHT = 4,
	parameter integer ARRAY_WIDTH = 4,
	parameter integer PROCESSOR_BITWIDTH = 16,	
    //GENERAL PARAMETERS
    parameter SYSTOLIC_WORD = PROCESSOR_BITWIDTH,
    parameter OPCODE_SIZE = 4,
    //SYSTOLIC PARAMETERS
    parameter DATA_IN_SIZE = ARRAY_HEIGHT * SYSTOLIC_WORD,
    parameter DATA_OUT_SIZE = ARRAY_WIDTH * SYSTOLIC_WORD,
    //PS SCRATCHPAD PARAMETERS
    parameter PS_SP_DEPTH = 4096,
    parameter PS_SP_ADDR_SIZE = $clog2(PS_SP_DEPTH),
    parameter PS_SP_WIDTH = SYSTOLIC_WORD * ARRAY_WIDTH,
    //ACTIVATION SCRATCHPAD PARAMETERS
    parameter ACT_SP_DEPTH = 4096,
    parameter ACT_SP_ADDR_SIZE = $clog2(ACT_SP_DEPTH),
    parameter ACT_SP_WIDTH = SYSTOLIC_WORD * ARRAY_HEIGHT,
    //BUSES PARAMETERS
    parameter BUS_INSTRUCTION_SIZE = 64,
    parameter BUS_WEIGHT_SIZE = SYSTOLIC_WORD * ARRAY_WIDTH,
    parameter BUS_ACTIVATION_SIZE = SYSTOLIC_WORD * ARRAY_HEIGHT,
    parameter BUS_PS_SIZE = SYSTOLIC_WORD * ARRAY_WIDTH,
    //MULTICASTING PARAMETERS
    parameter ID_ROW_BITWIDTH = $clog2(ARRAY_HEIGHT),
    parameter BUS_MULTICAST_IN_SIZE = BUS_WEIGHT_SIZE + ID_ROW_BITWIDTH,
    //DMA ADDRESS
    parameter DMA_ADDRESS_SIZE = 32
)(
    input logic                             clk,
    input logic                             rst,
    input logic                             enable,
    input logic [OPCODE_SIZE-1:0]           opcode,
    input logic [PS_SP_ADDR_SIZE-1:0]       instr_pacc_address_ps_1,
    input logic [PS_SP_ADDR_SIZE-1:0]       instr_pacc_address_ps_2,
    input logic [ACT_SP_ADDR_SIZE-1:0]      instr_mm_end_address_activation,  
    input logic [ACT_SP_ADDR_SIZE-1:0]      instr_mm_start_address_activation,  
    input logic [PS_SP_ADDR_SIZE-1:0]       instr_mm_address_ps,  
    input logic [ID_ROW_BITWIDTH-1:0]       instr_ws_row_id,
    input logic [ACT_SP_ADDR_SIZE-1:0]      instr_as_address_act,
    input logic [PS_SP_ADDR_SIZE-1:0]       instr_ps_address_ps,
    input logic [PS_SP_ADDR_SIZE-1:0]       instr_pc_address_ps,
    output logic [ACT_SP_ADDR_SIZE-1:0]     address_act,
    output logic [PS_SP_ADDR_SIZE-1:0]      address_ps_a,
    output logic [PS_SP_ADDR_SIZE-1:0]      address_ps_b,
    output logic [ID_ROW_BITWIDTH-1:0]      row_id,
    output logic                            mm_act_done,
    output logic                            ps_we,
    output logic                            op_done,
    output logic                            mm_pe_selector
    );
    
    logic [PS_SP_ADDR_SIZE-1:0]     cnt_coll;
    logic                           ps_done;
    logic [PS_SP_ADDR_SIZE-1:0]     cnt_acc;
    logic                           ps_we_r;
    
    //OPCODE CASTING
    opcode_enum_t opcode_enum;
    always_comb begin
        opcode_enum = opcode_enum_t'(opcode);
    end
    
    logic mm_act_done_r;
    logic mm_ps_we_r;
    logic mm_op_done_r;
    logic mm_pe_selector_r;
    logic [PS_SP_ADDR_SIZE-1:0]     instr_pacc_address_ps_1_r;
    logic [PS_SP_ADDR_SIZE-1:0]     instr_pacc_address_ps_2_r;
    logic [OPCODE_SIZE-1:0]         opcode_r;
    logic [PS_SP_ADDR_SIZE-1:0]     instr_pc_address_ps_r;
    
    logic [ACT_SP_ADDR_SIZE-1:0] moving_act;
    logic [ACT_SP_ADDR_SIZE-1:0] moving_ps;
    logic [ACT_SP_ADDR_SIZE-1:0] cnt_mm;
    logic [ACT_SP_ADDR_SIZE-1:0] addr_diff;
    
    assign addr_diff = instr_mm_end_address_activation - instr_mm_start_address_activation;
    
    always_ff @(posedge clk) begin
        if (rst || ~enable || (opcode_enum != MM_op && opcode_enum != PSUM_ACC_op && opcode_enum != PS_COLL_op)) begin
            moving_act <= '0;
            moving_ps <= '0;
            cnt_mm <= '0;
            cnt_acc <= '0;
            cnt_coll <= '0;
            mm_act_done_r <= 0;
            mm_ps_we_r <= 0;
            mm_op_done_r <= 0;
            mm_pe_selector_r <= 0;
            ps_we_r <= 0;
            ps_done <= 0;
            instr_pacc_address_ps_1_r <= instr_pacc_address_ps_1;
            instr_pacc_address_ps_2_r <= instr_pacc_address_ps_2; 
            opcode_r <= opcode;
            instr_pc_address_ps_r <= instr_pc_address_ps;
        end else begin
            case (opcode_enum)
                MM_op: begin
                    cnt_acc <= '0;
                    cnt_coll <= '0;
                    if (cnt_mm < ARRAY_HEIGHT) begin
                        mm_ps_we_r <= 0;
                        moving_ps <= instr_mm_address_ps;
                    end else if (cnt_mm < (addr_diff + ARRAY_HEIGHT) + 1)begin
                        mm_ps_we_r <= 1;
                        moving_ps <= instr_mm_address_ps + cnt_mm - ARRAY_HEIGHT;
                    end else begin
                        mm_ps_we_r <= 1;
                        moving_ps <= '0;
                    end
                    
                    if (cnt_mm < addr_diff + 1) begin
                        mm_act_done_r <= 0;
                        moving_act <= instr_mm_start_address_activation + cnt_mm;
                    end else begin
                        mm_act_done_r <= 1;
                        moving_act <= '0;
                    end
                    
                    if (cnt_mm > (addr_diff + ARRAY_HEIGHT)) begin
                        mm_op_done_r <= 1;
                        cnt_mm <= cnt_mm;
                    end else begin
                        mm_op_done_r <= 0;
                        cnt_mm <= cnt_mm + 1;
                    end
                    
                    if (cnt_mm >= 0) begin
                        mm_pe_selector_r <= 1;
                    end else begin
                        mm_pe_selector_r <= 0;
                    end
                    ps_done <= 0;
                    ps_we_r <= 0;
                    instr_pacc_address_ps_1_r <= instr_pacc_address_ps_1;
                    instr_pacc_address_ps_2_r <= instr_pacc_address_ps_2; 
                    opcode_r <= opcode;
                    instr_pc_address_ps_r <= instr_pc_address_ps;
                end
                PSUM_ACC_op: begin
                    moving_act <= moving_act;
                    moving_ps <= moving_ps;
                    mm_act_done_r <= mm_act_done_r;
                    mm_ps_we_r <= mm_ps_we_r;
                    mm_op_done_r <= mm_op_done_r;
                    mm_pe_selector_r <= mm_pe_selector_r;
                    instr_pacc_address_ps_1_r <= instr_pacc_address_ps_1;
                    instr_pacc_address_ps_2_r <= instr_pacc_address_ps_2; 
                    opcode_r <= opcode;
                    instr_pc_address_ps_r <= instr_pc_address_ps;
                    cnt_mm <= '0;
                    cnt_coll <= '0;
                    if (cnt_acc == 0) begin 
                        ps_done <= 0;
                        ps_we_r <= 1;
                        cnt_acc <= cnt_acc + 1;
                    end else if (cnt_acc == 1) begin 
                        ps_done <= 1;
                        ps_we_r <= 1;
                        cnt_acc <= cnt_acc + 1;
                    end else if (cnt_acc == 2) begin 
                        ps_done <= 1;
                        ps_we_r <= 0;
                        cnt_acc <= cnt_acc + 1;
                    end else if (cnt_acc > 2 && ((instr_pacc_address_ps_1_r != instr_pacc_address_ps_1)||(instr_pacc_address_ps_2_r != instr_pacc_address_ps_2)||(opcode_r != opcode))) begin
                        ps_done <= 0;
                        ps_we_r <= 0;
                        cnt_acc <= '0;
                    end
                    
                end
                PS_COLL_op: begin
                    moving_act <= moving_act;
                    moving_ps <= moving_ps;
                    mm_act_done_r <= mm_act_done_r;
                    mm_ps_we_r <= mm_ps_we_r;
                    mm_op_done_r <= mm_op_done_r;
                    mm_pe_selector_r <= mm_pe_selector_r;
                    instr_pacc_address_ps_1_r <= instr_pacc_address_ps_1;
                    instr_pacc_address_ps_2_r <= instr_pacc_address_ps_2; 
                    instr_pc_address_ps_r <= instr_pc_address_ps;
                    opcode_r <= opcode;
                    cnt_mm <= '0;
                    cnt_acc <= '0;
                    ps_we_r <= 0;
                    if (cnt_coll >= 0 && ((instr_pc_address_ps_r == instr_pc_address_ps)||(opcode_r == opcode))) begin 
                        ps_done <= 1;
                        cnt_coll <= cnt_coll + 1;
                    end else begin 
                        ps_done <= 0;
                        cnt_coll <= '0;
                    end 
                end
                default: begin
                    moving_act <= '0;
                    moving_ps <= '0;
                    cnt_mm <= '0;
                    cnt_acc <= '0;
                    cnt_coll <= '0;
                    mm_act_done_r <= 0;
                    mm_ps_we_r <= 0;
                    mm_op_done_r <= 0;
                    mm_pe_selector_r <= 0;
                    instr_pacc_address_ps_1_r <= instr_pacc_address_ps_1;
                    instr_pacc_address_ps_2_r <= instr_pacc_address_ps_2; 
                    opcode_r <= opcode;
                    instr_pc_address_ps_r <= instr_pc_address_ps;
                    ps_we_r <= 0;
                    ps_done <= 0;
                end
            endcase
        end
    end
    
    always_comb begin
        if (enable) begin
            case(opcode_enum)
                IDLE_op: begin
                    address_act = '0;
                    address_ps_a = '0;
                    address_ps_b = '0;
                    row_id = '0;
                    mm_act_done = 0;
                    ps_we = 0;
                    op_done = 0;
                    mm_pe_selector = 0;
                end
                WEIGHT_STORE_op: begin
                    address_act = '0;
                    address_ps_a = '0;
                    address_ps_b = '0;
                    row_id = instr_ws_row_id;
                    mm_act_done = 0;
                    ps_we = 0;
                    op_done = 0;
                    mm_pe_selector = 0;
                end
                ACT_STORE_op: begin
                    address_act = instr_as_address_act;
                    address_ps_a = '0;
                    address_ps_b = '0;
                    row_id = '0;
                    mm_act_done = 0;
                    ps_we = 0;
                    op_done = 0;
                    mm_pe_selector = 0;
                end
                PSUM_STORE_op: begin
                    address_act = '0;
                    address_ps_a = instr_ps_address_ps;
                    address_ps_b = '0;
                    row_id = '0;
                    mm_act_done = 0;
                    ps_we = ps_we_r;
                    op_done = ps_done;
                    mm_pe_selector = 0;
                end
                MM_op: begin
                    address_act = moving_act;
                    address_ps_a = moving_ps;
                    address_ps_b = '0;
                    row_id = '0;
                    mm_act_done = mm_act_done_r;
                    ps_we = mm_ps_we_r;
                    op_done = mm_op_done_r;
                    mm_pe_selector = mm_pe_selector_r;
                end
                PSUM_ACC_op: begin
                    address_act = '0;
                    address_ps_a = instr_pacc_address_ps_1;
                    address_ps_b = instr_pacc_address_ps_2;
                    row_id = '0;
                    mm_act_done = 0;
                    ps_we = ps_we_r;
                    if (ps_done &&((instr_pacc_address_ps_1_r != instr_pacc_address_ps_1)||(instr_pacc_address_ps_2_r != instr_pacc_address_ps_2)||(opcode_r != opcode))) begin
                        op_done = 0;
                    end else begin
                        op_done = ps_done;
                    end
                    mm_pe_selector = 0;
                end
                PS_COLL_op: begin
                    address_act = '0;
                    address_ps_a = instr_pc_address_ps;
                    address_ps_b = '0;
                    row_id = '0;
                    mm_act_done = 0;
                    ps_we = 0;
                    if (ps_done &&((instr_pc_address_ps_r != instr_pc_address_ps)||(opcode_r != opcode))) begin
                        op_done = 0;
                    end else begin
                        op_done = ps_done;
                    end
                    mm_pe_selector = 0;
                end
                default: begin
                    address_act = '0;
                    address_ps_a = '0;
                    address_ps_b = '0;
                    row_id = '0;
                    mm_act_done = 0;
                    ps_we = 0;
                    op_done = 0;
                    mm_pe_selector = 0;
                end
            endcase
        end else begin
           address_act = '0;
           address_ps_a = '0;
           address_ps_b = '0;
           row_id = '0;
           mm_act_done = 0;
           ps_we = 0;
           op_done = 0;
           mm_pe_selector = 0; 
        end    
    end
    
endmodule



