/*
Code Developed by Gabriel Oña
Dragon AI Accelerator - Proyecto Integrador
Date: 07/04/2025 Quito - Ecuador

Module Details:
-------------------------------------------------------------------------------
Module Name      : bus_adder
Project          : Dragon AI Accelerator
Description      : 
    The bus_adder module performs an element-wise addition of two partial sum 
    data buses, typically representing two rows of accumulated data within the 
    systolic array architecture. This module is essential for supporting 
    operations like partial sum accumulation during matrix multiplication 
    and data fusion stages.
Inputs:
  - bus_a         : First input bus containing partial sums.
  - bus_b         : Second input bus containing partial sums.

Outputs:
  - bus_out       : Output bus with element-wise addition results.
*/

`include "enums.sv"

module bus_adder#(
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
    input logic [BUS_PS_SIZE-1:0] bus_a,  
    input logic [BUS_PS_SIZE-1:0] bus_b,  
    output logic [BUS_PS_SIZE-1:0] bus_out 
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
    
    data_union_ps_t a;
    data_union_ps_t b;
    data_union_ps_t out;
    
    always_comb begin
        a.complete_bus = bus_a; 
        b.complete_bus = bus_b;

        for (int i = 0; i < ARRAY_WIDTH; i++) begin
            logic [SYSTOLIC_WORD:0] temp_result; // Extra bit for overflow
            temp_result = a.partition[i] + b.partition[i];
            out.partition[i] = temp_result[SYSTOLIC_WORD-1:0]; // Truncate to SYSTOLIC_WORD bits
        end
    end
    assign bus_out = out.complete_bus;
endmodule
