/*
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025 Quito - Ecuador  

Module Details:
-------------------------------------------------------------------------------
Module Name      : controller  
Project          : Dragon AI Accelerator  
Description      : 

  The controller module serves as the central control unit for the Dragon AI Accelerator.
  It coordinates various components and memory management within the accelerator, including
  activation and partial sum memory, weight storage, and DMA operations. This module manages
  the flow of data between different subsystems, controlling read and write operations to 
  various scratchpads, multicasting, and instruction dispatch. 

  It interfaces with the AXI Interconnect through an Slave AXI port configured as a custom 
  IP, which connects with the Processing System (PS) of the Zynq Ultrascale+ MPSoC. 
  Additionally, it handles Dragon AI Accelerator interrupt signal through the Generic Interrupt
  Controller (GIC) and managing DMA transactions.

Inputs:
  - clk                           : Clock signal.
  - rst                           : Reset signal.
  - enable                        : Global enable signal.
  - instr                         : Instruction bus (64 bits).
  - dma_data_in                   : Data input from DMA.
  - ps_collect_data               : Collected data from partial sum memory.
  - cu_to_mcst_ready              : Ready signals from compute units.
  
Outputs:
  - cu_to_mcst_data               : Data to multicast.
  - cu_to_mcst_id                 : Row ID for multicast.
  - cu_to_mcst_enable             : Enable multicast.
  - cu_to_act_control_en          : Enable signal for activation control.
  - cu_to_act_control_we          : Write enable for activation control.
  - cu_to_act_control_re          : Read enable for activation control.
  - cu_to_act_control_address     : Address for activation control.
  - cu_to_act_control_data        : Data for activation control.
  - cu_to_ps_control_en           : Enable signal for partial sum control.
  - cu_to_ps_control_we_a         : Write enable for partial sum control port A.
  - cu_to_ps_control_we_b         : Write enable for partial sum control port B.
  - cu_to_ps_control_re_a         : Read enable for partial sum control port A.
  - cu_to_ps_control_re_b         : Read enable for partial sum control port B.
  - cu_to_ps_control_address_a    : Address for partial sum control port A.
  - cu_to_ps_control_address_b    : Address for partial sum control port B.
  - cu_to_ps_control_data         : Data for partial sum control.
  - cu_to_mux_ps_control          : Multiplexer selector for PS control.
  - pe_sys_selector               : PE system selector.
  - dma_address                   : DMA address output.
  - dma_data_out                  : DMA data output.
  - instr_ready_signal            : Instruction ready signal(IRQ).
*/
`include "enums.sv"

module controller #(
	// Parameters Override from parent modules
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
    input logic                                 clk,
    input logic                                 rst,
    input logic                                 enable,
    input logic                                 axi_irq_clear,
    input logic                                 axi_irq_enable,
    input logic [BUS_INSTRUCTION_SIZE-1:0]      instr,
    input logic [DATA_IN_SIZE-1:0]              dma_data_in,
    input logic [DATA_OUT_SIZE-1:0]             ps_collect_data,
    //Interface between Control Unit and Multicast Elements
    input logic [ARRAY_HEIGHT-1:0]              cu_to_mcst_ready,
    output logic [BUS_WEIGHT_SIZE-1:0]          cu_to_mcst_data,
    output logic [ID_ROW_BITWIDTH-1:0]          cu_to_mcst_id,
    output logic                                cu_to_mcst_enable,
    //Interface between Control Unit and Activation Scratchpad 
    output logic                                cu_to_act_control_en,
    output logic                                cu_to_act_control_we,
    output logic                                cu_to_act_control_re,
    output logic [ACT_SP_ADDR_SIZE-1:0]         cu_to_act_control_address,
    output logic [ACT_SP_WIDTH-1:0]             cu_to_act_control_data,
    //Interface between Control Unit and Partial Sum Scratchpad
    output logic                                cu_to_ps_control_en,
    output logic                                cu_to_ps_control_we_a,
    output logic                                cu_to_ps_control_we_b,
    output logic                                cu_to_ps_control_re_a,
    output logic                                cu_to_ps_control_re_b,
    output logic [ACT_SP_ADDR_SIZE-1:0]         cu_to_ps_control_address_a,
    output logic [ACT_SP_ADDR_SIZE-1:0]         cu_to_ps_control_address_b,
    output logic [ACT_SP_WIDTH-1:0]             cu_to_ps_control_data,
    output MUX_PS_selector_t                    cu_to_mux_ps_control,
    output PE_selector_t                        pe_sys_selector,
    //Interface with Processing System Zynq SoC
    output logic [DMA_ADDRESS_SIZE-1:0]         dma_address,
    output logic [DATA_OUT_SIZE-1:0]            dma_data_out,
    output logic                                interrupt
    );
    /*START STRUCTS*/
    //SYSTOLIC STRUCTURES
        //UNION DEFINITIONS
    typedef union packed{
        logic [BUS_ACTIVATION_SIZE-1:0] complete_bus;
        logic [ARRAY_WIDTH-1:0][SYSTOLIC_WORD-1:0] partition;
    } data_union_activation_t;
    typedef union packed{
        logic [BUS_PS_SIZE-1:0] complete_bus;
        logic [ARRAY_WIDTH-1:0][SYSTOLIC_WORD-1:0] partition;
    } data_union_ps_t;
    typedef union packed{
        logic [BUS_WEIGHT_SIZE-1:0] complete_bus;
        logic [ARRAY_WIDTH-1:0][SYSTOLIC_WORD-1:0] partition;
    } data_union_weight_t;
    // DRAGON ACCELERATOR INPUT
        // STRUCT DEFINITIONS
    typedef struct packed{
        logic [OPCODE_SIZE-1:0]             opcode;
        logic [11:0]                        x1;
        logic [16 - ID_ROW_BITWIDTH - 1:0]  x2;
        logic [ID_ROW_BITWIDTH-1:0]         row_id;
        logic [DMA_ADDRESS_SIZE-1:0]        address_DMA;
    }weight_store_t;
    typedef struct packed{
        logic [OPCODE_SIZE-1:0]         opcode;
        logic [11:0]                    x1;
        logic [3:0]                     x2;
        logic [ACT_SP_ADDR_SIZE-1:0]    address_act;
        logic [DMA_ADDRESS_SIZE-1:0]    address_DMA;
    }activation_store_t;
    typedef struct packed{
        logic [OPCODE_SIZE-1:0]         opcode;
        logic [11:0]                    x1;
        logic [3:0]                     x2;
        logic [PS_SP_ADDR_SIZE-1:0]     address_ps;
        logic [DMA_ADDRESS_SIZE-1:0]    address_DMA;
    }partial_sum_store_t;
    typedef struct packed{
        logic [OPCODE_SIZE-1:0]         opcode;
        logic [11:0]                    x1;
        logic [3:0]                     x2;
        logic [ACT_SP_ADDR_SIZE-1:0]    end_address_activation;
        logic [3:0]                     x3;
        logic [ACT_SP_ADDR_SIZE-1:0]    start_address_activation;
        logic [3:0]                     x4;
        logic [PS_SP_ADDR_SIZE-1:0]     address_ps;
    }matrix_multiply_t;
    typedef struct packed{
        logic [OPCODE_SIZE-1:0]         opcode;
        logic [11:0]                    x1;
        logic [15:0]                    x2;
        logic [3:0]                     x3;
        logic [PS_SP_ADDR_SIZE-1:0]     address_ps_1;
        logic [3:0]                     x4;
        logic [PS_SP_ADDR_SIZE-1:0]     address_ps_2;
    }partial_sum_accumulate_t;
    typedef struct packed{
        logic [OPCODE_SIZE-1:0]         opcode;
        logic [11:0]                    x1;
        logic [3:0]                     x2;
        logic [PS_SP_ADDR_SIZE-1:0]     address_ps;
        logic [DMA_ADDRESS_SIZE-1:0]    address_DMA;
    }partial_sum_collect_t;
    typedef struct packed{
        logic [OPCODE_SIZE-1:0]                         opcode;
        logic [BUS_INSTRUCTION_SIZE - OPCODE_SIZE-1:0]  x;
    }opcode_identifier_t;
        //UNION DEFINITION
    typedef union{
        logic [BUS_INSTRUCTION_SIZE-1:0]    complete_bus;
        opcode_identifier_t                 opcode_fields;
        weight_store_t                      weight_store_fields;
        activation_store_t                  activation_store_fields;
        partial_sum_store_t                 partial_sum_store_fields;
        matrix_multiply_t                   matrix_multiply_fields;
        partial_sum_accumulate_t            partial_sum_accumulate_fields;
        partial_sum_collect_t               partial_sum_collect_fields;
    } data_union_in_accelerator_t;
    // CONTROL UNIT
        //STRUCT DEFINITIONS
    typedef struct packed{
        logic                               en;
        logic                               we;
        logic                               re;
        logic [ACT_SP_ADDR_SIZE-1:0]        address;
        logic [ACT_SP_WIDTH-1:0]            data;
    }cu_act_sp_control_t;
    typedef struct packed{
        logic                               en;
        logic                               we_a;
        logic                               we_b;
        logic                               re_a;
        logic                               re_b;
        logic [PS_SP_ADDR_SIZE-1:0]         address_a;
        logic [PS_SP_ADDR_SIZE-1:0]         address_b;
        logic [PS_SP_WIDTH-1:0]             data;
    }cu_ps_sp_control_t;
    /*END STRUCTS*/
    
    //CASTING Verilog
    data_union_in_accelerator_t instruction;
    always_comb begin
        instruction = data_union_in_accelerator_t'(instr);
    end
    
    logic mm_act_done;
    logic ps_we;
    logic op_done;
    logic mm_pe_selector;
    logic instr_ready_signal;
    
    main_decoder #(
	   // Parameters Override  
		.ARRAY_HEIGHT(ARRAY_HEIGHT),
		.ARRAY_WIDTH(ARRAY_WIDTH),
		.PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
	) my_main_deco (
	    .enable(enable),
        .opcode(instruction.opcode_fields.opcode),
        .sys_ready(&cu_to_mcst_ready),
        .mm_act_done(mm_act_done),
        .ps_we(ps_we),
        .op_done(op_done),
        .mm_pe_selector(mm_pe_selector),
        .pe_selector(pe_sys_selector),
        .mux_acc_selector(cu_to_mux_ps_control),
        .enable_mcst(cu_to_mcst_enable),
        .enable_act(cu_to_act_control_en),
        .we_act(cu_to_act_control_we),
        .re_act(cu_to_act_control_re),
        .enable_ps(cu_to_ps_control_en),
        .we_a_ps(cu_to_ps_control_we_a),
        .we_b_ps(cu_to_ps_control_we_b),
        .re_a_ps(cu_to_ps_control_re_a),
        .re_b_ps(cu_to_ps_control_re_b),
        .done_instr(instr_ready_signal)
    );
    
    addresses_manager #(
	   // Parameters Override  
		.ARRAY_HEIGHT(ARRAY_HEIGHT),
		.ARRAY_WIDTH(ARRAY_WIDTH),
		.PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
	) my_addr_manager(
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .opcode(instruction.opcode_fields.opcode),
        .instr_pacc_address_ps_1(instruction.partial_sum_accumulate_fields.address_ps_1),
        .instr_pacc_address_ps_2(instruction.partial_sum_accumulate_fields.address_ps_2),
        .instr_mm_end_address_activation(instruction.matrix_multiply_fields.end_address_activation),
        .instr_mm_start_address_activation(instruction.matrix_multiply_fields.start_address_activation),
        .instr_mm_address_ps(instruction.matrix_multiply_fields.address_ps),
        .instr_ws_row_id(instruction.weight_store_fields.row_id),
        .instr_as_address_act(instruction.activation_store_fields.address_act),
        .instr_ps_address_ps(instruction.partial_sum_store_fields.address_ps),
        .instr_pc_address_ps(instruction.partial_sum_collect_fields.address_ps),
        .address_act(cu_to_act_control_address),
        .address_ps_a(cu_to_ps_control_address_a),
        .address_ps_b(cu_to_ps_control_address_b),
        .row_id(cu_to_mcst_id),
        .mm_act_done(mm_act_done),
        .ps_we(ps_we),
        .op_done(op_done),
        .mm_pe_selector(mm_pe_selector)
    );
    
    dma_engine #(
	   // Parameters Override  
		.ARRAY_HEIGHT(ARRAY_HEIGHT),
		.ARRAY_WIDTH(ARRAY_WIDTH),
		.PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
	) my_dma_eng(
	    .enable(enable),
        .opcode(instruction.opcode_fields.opcode),
        .deco_address(instruction.weight_store_fields.address_DMA),
        .dma_data_in(dma_data_in),
        .ps_collect_data(ps_collect_data),
        .dma_address(dma_address),
        .act_data(cu_to_act_control_data),
        .ps_data(cu_to_ps_control_data),
        .weight_data(cu_to_mcst_data),
        .dma_data_out(dma_data_out)
    );
    
    interrupt_controller my_irq_eng(
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .instruction_done(instr_ready_signal),
        .axi_irq_clear(axi_irq_clear),
        .axi_irq_enable(axi_irq_enable),
        .interrupt(interrupt)
    );
    
endmodule
