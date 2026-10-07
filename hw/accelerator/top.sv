/* 
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025 Quito - Ecuador  

Module Details:
-------------------------------------------------------------------------------
Module Name      : top
Project          : Dragon AI Accelerator
Description      : 
  The top module integrates all components of the Dragon AI Accelerator.
  It includes a systolic array for deep learning operations, memory scratchpads 
  for activation and partial sums, and a control unit that generates the necessary 
  signals to orchestrate operations. The module interfaces with multicast controllers, 
  which manage memory access for weights storing, and a partial sum adder to accumulate
  matrix multiplication result and partial sum accumulation.
  
Inputs:
  - clk                       : Clock signal.
  - rst                       : Reset signal.
  - enable                    : Enables the module's operation.
  - bus_instr                 : Instruction bus containing opcode and parameters.
  - dma_data_in               : Data input for the Direct Memory Access (DMA).
  
Outputs:
  - dma_address               : Output address for DMA.
  - dma_data_out              : Data output for DMA.
  - ready_interrupt           : Indicates if the module is ready for the next operation.
*/

`include "enums.sv"

module top#(
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
    input logic                                 clk,
    input logic                                 rst,
    input logic                                 enable,
    input logic                                 axi_irq_enable,
    input logic                                 axi_irq_clear,
    input logic [BUS_INSTRUCTION_SIZE-1:0]      bus_instr,
    input logic [DATA_IN_SIZE-1:0]              dma_data_in,
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
        //INTERFACE SYSTOLIC to MCST
    interface sys_to_mcst;
        data_union_weight_t [ARRAY_HEIGHT-1:0]  weight_rows;
        logic [ARRAY_HEIGHT-1:0]                ready;
        logic [ARRAY_HEIGHT-1:0]                enable;
        modport master(
            input ready,
            output weight_rows,
            output enable
        );
        modport slave(
            input weight_rows,
            input enable,
            output ready
        );
    endinterface 
    //MULTICAST STRUCTURES
        //INTERFACE DEFINITIONS
    interface control_to_mcst;
        data_union_weight_t             data;
        logic [ID_ROW_BITWIDTH-1:0]     id;
        logic                           enable;
        logic [ARRAY_HEIGHT-1:0]        ready;
        modport master(
            input ready,
            output data,
            output id,
            output enable
        );
        modport slave(
            input data,
            input id,
            input enable,
            output ready
        );
    endinterface 
    // DRAGON ACCELERATOR INPUT
        // STRUCT DEFINITIONS
    typedef struct packed{
        logic [OPCODE_SIZE-1:0]             opcode;
        logic [11:0]                        x1;
        logic [16 - ID_ROW_BITWIDTH:0]      x2;
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
        instruction = data_union_in_accelerator_t'(bus_instr);
    end
    
    //BUS INTERFACES
    sys_to_mcst         bus_sys_to_mcst();
    control_to_mcst     bus_cu_to_mcst();
    
    //ACT VARIABLES
    data_union_activation_t act_sp_out;
    assign activation_bus = act_sp_out.complete_bus;
    
    //PS VARIABLES
    data_union_ps_t ps_sys_out_bus;
    assign ps_bus = ps_sys_out_bus.complete_bus;
    
    data_union_ps_t ps_mux_to_ps;
    data_union_ps_t ps_alu_to_mux;
    data_union_ps_t ps_sp_data_1;
    data_union_ps_t ps_sp_data_2;
    logic [DATA_OUT_SIZE-1:0] result_data_out;
    
    //CONTROL SIGNALS
    MUX_PS_selector_t mux_ps_control;
    cu_act_sp_control_t cu_to_act_control;
    cu_ps_sp_control_t cu_to_ps_control;
    PE_selector_t       PE_selector;
    
    //CONTROLLER UNIT
    controller #(
	   // Parameters Override  
		.ARRAY_HEIGHT(ARRAY_HEIGHT),
		.ARRAY_WIDTH(ARRAY_WIDTH),
		.PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
	)control_unit(
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .instr(bus_instr),
        .axi_irq_clear(axi_irq_clear),
        .axi_irq_enable(axi_irq_enable),
        .dma_data_in(dma_data_in),
        .ps_collect_data(result_data_out),
        .cu_to_mcst_ready(bus_cu_to_mcst.ready),
        .cu_to_mcst_data(bus_cu_to_mcst.data),
        .cu_to_mcst_id(bus_cu_to_mcst.id),
        .cu_to_mcst_enable(bus_cu_to_mcst.enable),
        .cu_to_act_control_en(cu_to_act_control.en),
        .cu_to_act_control_we(cu_to_act_control.we),
        .cu_to_act_control_re(cu_to_act_control.re),
        .cu_to_act_control_address(cu_to_act_control.address),
        .cu_to_act_control_data(cu_to_act_control.data),
        .cu_to_ps_control_en(cu_to_ps_control.en),
        .cu_to_ps_control_we_a(cu_to_ps_control.we_a),
        .cu_to_ps_control_we_b(cu_to_ps_control.we_b),
        .cu_to_ps_control_re_a(cu_to_ps_control.re_a),
        .cu_to_ps_control_re_b(cu_to_ps_control.re_b),
        .cu_to_ps_control_address_a(cu_to_ps_control.address_a),
        .cu_to_ps_control_address_b(cu_to_ps_control.address_b),
        .cu_to_ps_control_data(cu_to_ps_control.data),
        .cu_to_mux_ps_control(mux_ps_control),
        .pe_sys_selector(PE_selector),
        .dma_address(dma_address),
        .dma_data_out(dma_data_out),
        .interrupt(interrupt)
    );
   
    //MEMORY SCRATCHPADS
        //PARTIAL SUM SCRATCHPAD DUAL 
    ram_dp #(
        .WIDTH(PS_SP_WIDTH),
        .DEPTH(PS_SP_DEPTH)
    )ps_scratchpad(
        .clk(clk),
        .en(cu_to_ps_control.en),
        .we_a(cu_to_ps_control.we_a),
        .re_a(cu_to_ps_control.re_a),
        .addr_a(cu_to_ps_control.address_a),
        .data_in_a(ps_mux_to_ps.complete_bus),
        .data_out_a(ps_sp_data_1.complete_bus),
        .we_b(cu_to_ps_control.we_b),
        .re_b(cu_to_ps_control.re_b),
        .addr_b(cu_to_ps_control.address_b),
        .data_in_b(),
        .data_out_b(ps_sp_data_2.complete_bus)
    );
        //ACTIVATION SCRATCHPAD
    ram_sp #(
        .WIDTH(ACT_SP_WIDTH),
        .DEPTH(ACT_SP_DEPTH)
    )act_scratchpad(
        .clk(clk),
        .en(cu_to_act_control.en),
        .we_a(cu_to_act_control.we),
        .re_a(cu_to_act_control.re),
        .addr_a(cu_to_act_control.address),
        .data_in_a(cu_to_act_control.data),
        .data_out_a(act_sp_out)
    );
   
    //SYSTOLIC ARRAY INSTANCE
    systolic_array  #(
	   // Parameters Override  
		.ARRAY_HEIGHT(ARRAY_HEIGHT),
		.ARRAY_WIDTH(ARRAY_WIDTH),
		.PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
	)sys_array (
        .clk(clk),
        .rst(rst),
        .PE_selector(PE_selector),
        .sys_to_mcst_bus_weight_rows(bus_sys_to_mcst.weight_rows),
        .sys_to_mcst_bus_enable(bus_sys_to_mcst.enable),
        .sys_to_mcst_bus_ready(bus_sys_to_mcst.ready),
        .activation_bus(act_sp_out.complete_bus),
        .ps_bus(ps_sys_out_bus.complete_bus)
    );
   
    //MULTICAST CONTROLLER INSTANCES
    for (genvar i = 0; i < ARRAY_HEIGHT; i++) begin
        logic [ID_ROW_BITWIDTH-1:0] id;
        assign id = i;
        multicast_controller #(
           // Parameters Override  
            .ARRAY_HEIGHT(ARRAY_HEIGHT),
            .ARRAY_WIDTH(ARRAY_WIDTH),
            .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
	   )mcst_inst(
            .id_cfg(id),
            .ready_from_sys(bus_sys_to_mcst.ready[i]),
            .enable_from_cu(bus_cu_to_mcst.enable),
            .id(bus_cu_to_mcst.id),
            .data_in(bus_cu_to_mcst.data.complete_bus),
            .data_out(bus_sys_to_mcst.weight_rows[i].complete_bus),
            .ready_to_cu(bus_cu_to_mcst.ready[i]),
            .enable_to_sys(bus_sys_to_mcst.enable[i])
        );
    end
    
    //MUX SYS TO PS
    always_comb begin
        case(mux_ps_control)
            MUX_SYSTOLIC: ps_mux_to_ps = ps_sys_out_bus;
            MUX_ALUS: ps_mux_to_ps = ps_alu_to_mux;
            default: ps_mux_to_ps = '0; 
        endcase
    end
   
   //Accumulator Partial Sum
    bus_adder #(
	   // Parameters Override  
		.ARRAY_HEIGHT(ARRAY_HEIGHT),
		.ARRAY_WIDTH(ARRAY_WIDTH),
		.PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
	)bus_adder_inst(
        .bus_a(ps_sp_data_1.complete_bus),
        .bus_b(ps_sp_data_2.complete_bus),
        .bus_out(ps_alu_to_mux.complete_bus)
    );
    
    //Output Return Data
    always_comb begin
        case(instruction.opcode_fields.opcode)
            PS_COLL_op: result_data_out = ps_sp_data_1.complete_bus;
            default: result_data_out = '0;
        endcase
    end
endmodule