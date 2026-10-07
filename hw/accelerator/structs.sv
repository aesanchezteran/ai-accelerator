`include "config.vh"

`ifndef STRUCTS
`define STRUCTS

/*
This file is dedicated to declare all the data structures in the project.
Written by Gabriel Ona
*/
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
    
    
`endif
