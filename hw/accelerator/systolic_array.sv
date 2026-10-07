/*
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025 Quito - Ecuador

Module Details:
-------------------------------------------------------------------------------
Module Name      : systolic_array
Project          : Dragon AI Accelerator
Description      : 
  The `systolic_array` module coordinates a 2D grid of Processing Elements (PEs) 
  to perform matrix-matrix multiplication using a weight-stationary dataflow. 
  It receives input activations and weights, distributes them across the array, 
  and collects the computed partial sums.
  
  This modular and scalable design is well-suited for deep learning workloads, 
  particularly matrix multiplications commonly found in neural network layers. 
  Both the array dimensions and the bitwidth of each data element are scalable,
  allowing the programmer to adapt to various performance and resource constraints.
  
Inputs:
  - clk                         : Clock signal.
  - rst                         : Reset signal.
  - PE_selector                 : Selector signal for PE operation mode.
  - sys_to_mcst_bus_weight_rows : Weight inputs for each row (multicast format).
  - sys_to_mcst_bus_enable      : Enable signals for weight loading into each row.
  - activation_bus              : Packed activations entering the array.

Outputs:
  - sys_to_mcst_bus_ready       : Ready signals from each PE row.
  - ps_bus                      : Packed partial sums output from the array.
*/

`include "enums.sv"

module systolic_array #(
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
    input logic                                             clk,
    input logic                                             rst,
    input PE_selector_t                                     PE_selector,
    //Systolic Array to Multicast Interface
    input logic [ARRAY_HEIGHT-1:0][BUS_WEIGHT_SIZE-1:0]     sys_to_mcst_bus_weight_rows,
    input logic [ARRAY_HEIGHT-1:0]                          sys_to_mcst_bus_enable,
    output logic [ARRAY_HEIGHT-1:0]                         sys_to_mcst_bus_ready,
    //Data Buses
    input logic [BUS_ACTIVATION_SIZE-1:0]                   activation_bus,
    output logic [BUS_PS_SIZE-1:0]                          ps_bus
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
    data_union_activation_t activation_bus_r;
    always_comb begin
        activation_bus_r = data_union_activation_t'(activation_bus);
    end
    data_union_ps_t ps_bus_r;
    always_comb begin
        ps_bus_r = data_union_ps_t'(ps_bus);
    end

    //Interconnection between PE and Ready Signals
    logic [ARRAY_WIDTH-1:0] ready_signals [ARRAY_HEIGHT];
    logic [SYSTOLIC_WORD-1:0] rows_wires [ARRAY_HEIGHT-1:0][ARRAY_WIDTH-2:0];
    logic [SYSTOLIC_WORD-1:0] columns_wires [ARRAY_HEIGHT-2:0][ARRAY_WIDTH-1:0];   
    
    //Generation of the array
    for (genvar i = 0; i < ARRAY_HEIGHT; i++) begin
        for (genvar j = 0; j < ARRAY_WIDTH; j++) begin
            data_union_weight_t weight;
            assign weight.complete_bus = sys_to_mcst_bus_weight_rows[i]; 
            //First PE(0,0)
            if (i == 0 && j == 0) begin
                pe#(
                   // Parameters Override  
                    .ARRAY_HEIGHT(ARRAY_HEIGHT),
                    .ARRAY_WIDTH(ARRAY_WIDTH),
                    .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
                ) PE_inst(
                    .clk(clk),
                    .rst(rst),
                    .selector(PE_selector),
                    .we_weight(sys_to_mcst_bus_enable[i]),
                    .n_weight(weight.partition[j]),
                    .activation(activation_bus_r.partition[i]),
                    .ps_in('0),
                    .out_activation(rows_wires[i][j]),
                    .out_ps(columns_wires[i][j]),
                    .ready(ready_signals[i][j])
                );
            // Middle PE(0,1:2)
            end else if (i == 0 && j < ARRAY_WIDTH-1 && j > 0) begin
                pe #(
                   // Parameters Override  
                    .ARRAY_HEIGHT(ARRAY_HEIGHT),
                    .ARRAY_WIDTH(ARRAY_WIDTH),
                    .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
                )PE_inst(
                    .clk(clk),
                    .rst(rst),
                    .selector(PE_selector),
                    .we_weight(sys_to_mcst_bus_enable[i]),
                    .n_weight(weight.partition[j]),
                    .activation(rows_wires[i][j-1]),
                    .ps_in('0),
                    .out_activation(rows_wires[i][j]),
                    .out_ps(columns_wires[i][j]),
                    .ready(ready_signals[i][j])
                );
            // Final PE(0,3)
            end else if (i == 0 && j == ARRAY_WIDTH-1) begin
                pe #(
                   // Parameters Override  
                    .ARRAY_HEIGHT(ARRAY_HEIGHT),
                    .ARRAY_WIDTH(ARRAY_WIDTH),
                    .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
                )PE_inst(
                    .clk(clk),
                    .rst(rst),
                    .selector(PE_selector),
                    .we_weight(sys_to_mcst_bus_enable[i]),
                    .n_weight(weight.partition[j]),
                    .activation(rows_wires[i][j-1]),
                    .ps_in('0),
                    .out_activation(),
                    .out_ps(columns_wires[i][j]),
                    .ready(ready_signals[i][j])
                );
            // First Row intermediate PE(1:2,0)
            end else if (j == 0 && i < ARRAY_HEIGHT-1 && i > 0) begin
                pe #(
                   // Parameters Override  
                    .ARRAY_HEIGHT(ARRAY_HEIGHT),
                    .ARRAY_WIDTH(ARRAY_WIDTH),
                    .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
                )PE_inst(
                    .clk(clk),
                    .rst(rst),
                    .selector(PE_selector),
                    .we_weight(sys_to_mcst_bus_enable[i]),
                    .n_weight(weight.partition[j]),
                    .activation(activation_bus_r.partition[i]),
                    .ps_in(columns_wires[i-1][j]),
                    .out_activation(rows_wires[i][j]),
                    .out_ps(columns_wires[i][j]),
                    .ready(ready_signals[i][j])
                );
           // Row Final Column 0 PE(3,0)
           end else if (j == 0 && i == ARRAY_HEIGHT-1) begin
                pe #(
                   // Parameters Override  
                    .ARRAY_HEIGHT(ARRAY_HEIGHT),
                    .ARRAY_WIDTH(ARRAY_WIDTH),
                    .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
                )PE_inst(
                    .clk(clk),
                    .rst(rst),
                    .selector(PE_selector),
                    .we_weight(sys_to_mcst_bus_enable[i]),
                    .n_weight(weight.partition[j]),
                    .activation(activation_bus_r.partition[i]),
                    .ps_in(columns_wires[i-1][j]),
                    .out_activation(rows_wires[i][j]),
                    .out_ps(ps_bus_r.partition[j]),
                    .ready(ready_signals[i][j])
                );
            // Row Middle Column Final PE(1:2,3)
            end else if (j == ARRAY_WIDTH-1 && i > 0 && i < ARRAY_HEIGHT-1) begin
                pe #(
                   // Parameters Override  
                    .ARRAY_HEIGHT(ARRAY_HEIGHT),
                    .ARRAY_WIDTH(ARRAY_WIDTH),
                    .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
                )PE_inst(
                    .clk(clk),
                    .rst(rst),
                    .selector(PE_selector),
                    .we_weight(sys_to_mcst_bus_enable[i]),
                    .n_weight(weight.partition[j]),
                    .activation(rows_wires[i][j-1]),
                    .ps_in(columns_wires[i-1][j]),
                    .out_activation(),
                    .out_ps(columns_wires[i][j]),
                    .ready(ready_signals[i][j])
                );
            // Row Final Column middle PE(3,1:2)
            end else if (i == ARRAY_HEIGHT-1 && j > 0 && j < ARRAY_WIDTH-1) begin
                pe #(
                   // Parameters Override  
                    .ARRAY_HEIGHT(ARRAY_HEIGHT),
                    .ARRAY_WIDTH(ARRAY_WIDTH),
                    .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
                )PE_inst(
                    .clk(clk),
                    .rst(rst),
                    .selector(PE_selector),
                    .we_weight(sys_to_mcst_bus_enable[i]),
                    .n_weight(weight.partition[j]),
                    .activation(rows_wires[i][j-1]),
                    .ps_in(columns_wires[i-1][j]),
                    .out_activation(rows_wires[i][j]),
                    .out_ps(ps_bus_r.partition[j]),
                    .ready(ready_signals[i][j])
                );
            // PE(3,3)
            end else if (i == ARRAY_HEIGHT-1 && j == ARRAY_WIDTH-1) begin
                pe #(
                   // Parameters Override  
                    .ARRAY_HEIGHT(ARRAY_HEIGHT),
                    .ARRAY_WIDTH(ARRAY_WIDTH),
                    .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
                )PE_inst(
                    .clk(clk),
                    .rst(rst),
                    .selector(PE_selector),
                    .we_weight(sys_to_mcst_bus_enable[i]),
                    .n_weight(weight.partition[j]),
                    .activation(rows_wires[i][j-1]),
                    .ps_in(columns_wires[i-1][j]),
                    .out_activation(),
                    .out_ps(ps_bus_r.partition[j]),
                    .ready(ready_signals[i][j])
                );
            // PE(1:2,1:2) 
            end else if (i > 0 && i < ARRAY_HEIGHT-1 && j > 0 && j < ARRAY_WIDTH-1) begin
                pe #(
                   // Parameters Override  
                    .ARRAY_HEIGHT(ARRAY_HEIGHT),
                    .ARRAY_WIDTH(ARRAY_WIDTH),
                    .PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
                )PE_inst(
                    .clk(clk),
                    .rst(rst),
                    .selector(PE_selector),
                    .we_weight(sys_to_mcst_bus_enable[i]),
                    .n_weight(weight.partition[j]),
                    .activation(rows_wires[i][j-1]),
                    .ps_in(columns_wires[i-1][j]),
                    .out_activation(rows_wires[i][j]),
                    .out_ps(columns_wires[i][j]),
                    .ready(ready_signals[i][j])
                );
            end
        end
    end
    //Ready signals from all rows with a reduction AND
    for (genvar i = 0; i < ARRAY_HEIGHT; i++) begin
        assign sys_to_mcst_bus_ready[i] = &ready_signals[i];
        assign ps_bus = ps_bus_r.complete_bus;
    end
endmodule
