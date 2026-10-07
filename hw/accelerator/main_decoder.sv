/* 
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025 Quito - Ecuador  

Module Details:
-------------------------------------------------------------------------------
Module Name      : main_decoder
Project          : Dragon AI Accelerator
Description      : 
  The `main_decoder` module decodes the opcode and generates control signals
  for the various components of the Dragon AI Accelerator. This module 
  coordinates different parts of the accelerator such as memory access write 
  and read enables, activation storage, and partial sum operations. It provides 
  control signals for managing the PE (Processing Element) selection, memory 
  addressing, and synchronization of different operations.
  
Inputs:
  - enable                  : Enables the module's operation.
  - opcode                  : The opcode for the current operation based on the Dragon AI ISA.
  - sys_ready               : Indicates if the systolic array is ready for the next operation.
  - mm_act_done             : Indicates if the matrix multiplication activation is complete.
  - ps_we                   : Write enable signal for partial sum.
  - op_done                 : Indicates if the current operation is complete.
  - mm_pe_selector          : Selector for matrix multiplication PE.

Outputs:
  - pe_selector             : Selector for the Processing Element.
  - mux_acc_selector        : Selector for the accumulator multiplexer.
  - enable_mcst             : Enables the multicast operation.
  - enable_act              : Enables activation store.
  - we_act                  : Write enable for the activation memory.
  - re_act                  : Read enable for the activation memory.
  - enable_ps               : Enables the partial sum operations.
  - we_a_ps                 : Write enable for partial sum A.
  - we_b_ps                 : Write enable for partial sum B.
  - re_a_ps                 : Read enable for partial sum A.
  - re_b_ps                 : Read enable for partial sum B.
  - done_instr              : Indicates if the instruction has been completed (IRQ - Interruption for General
                              Interrupt Controller (GIC).
*/
`include "enums.sv"

module main_decoder#(
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
    input logic                     enable,
    input logic [OPCODE_SIZE-1:0]   opcode,
    input logic                     sys_ready,
    input logic                     mm_act_done,
    input logic                     ps_we,
    input logic                     op_done,
    input logic                     mm_pe_selector,
    output PE_selector_t            pe_selector,
    output MUX_PS_selector_t        mux_acc_selector,
    output logic                    enable_mcst,
    output logic                    enable_act,
    output logic                    we_act,
    output logic                    re_act,
    output logic                    enable_ps,
    output logic                    we_a_ps,
    output logic                    we_b_ps,
    output logic                    re_a_ps,
    output logic                    re_b_ps,
    output logic                    done_instr
    );
    //OPCODE CASTING
    opcode_enum_t opcode_enum;
    always_comb begin
        opcode_enum = opcode_enum_t'(opcode);
    end
    
    always_comb begin
        if (enable) begin
            case(opcode_enum)
                IDLE_op: begin
                    pe_selector = PE_IDLE;
                    mux_acc_selector = MUX_SYSTOLIC;
                    enable_mcst = 0;
                    enable_act = 0;
                    we_act = 0;
                    re_act = 0;
                    enable_ps = 0;
                    we_a_ps = 0;
                    we_b_ps = 0;
                    re_a_ps = 0;
                    re_b_ps = 0;
                    done_instr = 0;
                end
                WEIGHT_STORE_op: begin
                    pe_selector = PE_WEIGHT_FETCH;
                    mux_acc_selector = MUX_SYSTOLIC;
                    if (sys_ready) begin
                        enable_mcst = 1;
                    end else begin
                        enable_mcst = 0;
                    end
                    enable_act = 0;
                    we_act = 0;
                    re_act = 0;
                    enable_ps = 0;
                    we_a_ps = 0;
                    we_b_ps = 0;
                    re_a_ps = 0;
                    re_b_ps = 0;
                    done_instr = 1;
                end
                ACT_STORE_op: begin
                    pe_selector = PE_IDLE;
                    mux_acc_selector = MUX_SYSTOLIC;
                    enable_mcst = 0;
                    enable_act = 1;
                    we_act = 1;
                    re_act = 0;
                    enable_ps = 0;
                    we_a_ps = 0;
                    we_b_ps = 0;
                    re_a_ps = 0;
                    re_b_ps = 0;
                    done_instr = 1;
                end
                PSUM_STORE_op: begin
                    pe_selector = PE_IDLE;
                    mux_acc_selector = MUX_SYSTOLIC;
                    enable_mcst = 0;
                    enable_act = 0;
                    we_act = 0;
                    re_act = 0;
                    enable_ps = 1;
                    we_a_ps = 1;
                    we_b_ps = 0;
                    re_a_ps = 0;
                    re_b_ps = 0;
                    done_instr = 1;
                end
                MM_op: begin
                    pe_selector = mm_pe_selector ? PE_MULTIPLY: PE_IDLE;
                    mux_acc_selector = MUX_SYSTOLIC;
                    enable_mcst = 0;
                    if (mm_act_done) begin
                        enable_act = 0;
                    end else begin
                        enable_act = 1;
                    end
                    we_act = 0;
                    re_act = 1;
                    enable_ps = 1;
                    we_b_ps = 0;
                    re_a_ps = 0;
                    re_b_ps = 0;
                    if (op_done) begin
                        done_instr = 1;
                        we_a_ps = 0;
                    end else begin
                        done_instr = 0;
                        we_a_ps = 1;
                    end
                end
                PSUM_ACC_op: begin
                    pe_selector = PE_IDLE;
                    mux_acc_selector = MUX_ALUS;
                    enable_mcst = 0;
                    enable_act = 0;
                    we_act = 0;
                    re_act = 0;
                    enable_ps = 1;
                    we_a_ps = ps_we;
                    we_b_ps = 0;
                    re_a_ps = 1;
                    re_b_ps = 1;
                    if (op_done) begin
                        done_instr = 1;
                    end else begin
                        done_instr = 0;
                    end
                end
                PS_COLL_op: begin
                    pe_selector = PE_IDLE;
                    mux_acc_selector = MUX_SYSTOLIC;
                    enable_mcst = 0;
                    enable_act = 0;
                    we_act = 0;
                    re_act = 0;
                    enable_ps = 1;
                    we_a_ps = 0;
                    we_b_ps = 0;
                    re_a_ps = 1;
                    re_b_ps = 0;
                    if (op_done) begin
                        done_instr = 1;
                    end else begin
                        done_instr = 0;
                    end
                end
                default: begin
                    pe_selector = PE_IDLE;
                    mux_acc_selector = MUX_SYSTOLIC;
                    enable_mcst = 0;
                    enable_act = 0;
                    we_act = 0;
                    re_act = 0;
                    enable_ps = 0;
                    we_a_ps = 0;
                    we_b_ps = 0;
                    re_a_ps = 0;
                    re_b_ps = 0;
                    done_instr = 0;
                end
            endcase
        end else begin
            pe_selector = PE_IDLE;
            mux_acc_selector = MUX_SYSTOLIC;
            enable_mcst = 0;
            enable_act = 0;
            we_act = 0;
            re_act = 0;
            enable_ps = 0;
            we_a_ps = 0;
            we_b_ps = 0;
            re_a_ps = 0;
            re_b_ps = 0;
            done_instr = 0;
        end
    end
endmodule
