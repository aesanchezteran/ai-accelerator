/*
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025 Quito - Ecuador

Module Details:
-------------------------------------------------------------------------------
Module Name      : pe
Project          : Dragon AI Accelerator
Description      : 
  The `pe` (Processing Element) module implements the core operation unit of 
  the Dragon AI Accelerator's systolic array. Each PE performs a Multiply-and-
  Accumulate (MAC) operation between an input activation, a stored weight, and 
  an incoming partial sum. It supports dynamic weight loading and pipelined 
  MAC computation driven by control logic.
  
  This module follows a weight-stationary dataflow, meaning weights are 
  loaded and kept stationary within the PE while activations and partial sums 
  flow through. This improves efficiency by minimizing the movement of weights 
  and reusing them across multiple cycles of computation, which is particularly 
  effective for deep learning inference workloads.  
  
Inputs:
  - clk              : Clock signal.
  - rst              : Reset signal.
  - selector         : Control selector to define PE operation mode.
  - we_weight        : Weight write enable.
  - n_weight         : New weight to load into internal register.
  - activation       : Activation value for multiplication.
  - ps_in            : Incoming partial sum to accumulate.
  
Outputs:
  - out_activation   : Forwarded activation value for next PE stage.
  - out_ps           : Output result of MAC operation.
  - ready            : Indicates PE is ready for next command.
*/

`include "enums.sv"

module pe#(
        // Users to add parameters here
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
		// User parameters ends
    )(
    input   logic                       clk,
    input   logic                       rst,
    input   PE_selector_t               selector,
    input   logic                       we_weight,
    input   logic [SYSTOLIC_WORD-1:0]   n_weight,
    input   logic [SYSTOLIC_WORD-1:0]   activation,
    input   logic [SYSTOLIC_WORD-1:0]   ps_in,
    output  logic [SYSTOLIC_WORD-1:0]   out_activation,
    output  logic [SYSTOLIC_WORD-1:0]   out_ps,
    output  logic                       ready
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
    
    //WIRES  CONNECTIONS
    logic [SYSTOLIC_WORD-1:0] weight_reg;
    logic [SYSTOLIC_WORD-1:0] ps_r;
    logic [SYSTOLIC_WORD-1:0] prod;
    
    //OPERATIONS
    always_comb begin
        prod = activation * weight_reg;
        ps_r = prod + ps_in; 
    end
    
    always_ff @(posedge clk) begin
        if (rst) begin
            weight_reg <= 0;
            out_ps <= 0;
            out_activation <= 0;
            ready <= 0;
        end else begin
            //SELECTOR PE 
            case (selector)
                PE_IDLE: begin
                    out_ps <= 0;
                    out_activation <= 0;
                    ready <= 1;
                end
                //If enable is on, fetch new weight
                PE_WEIGHT_FETCH: begin
                    out_ps <= 0;
                    out_activation <= 0;
                    if (we_weight) begin
                        weight_reg <= n_weight;
                    end
                    ready <= 1;
                end 
                PE_MULTIPLY: begin
                    out_ps <= ps_r;
                    out_activation <= activation;
                    ready <= 0;
                end
                default: begin
                    out_ps <= 0;
                    out_activation <= 0;
                    ready <= 1;
                end
            endcase
        end
    end
endmodule
