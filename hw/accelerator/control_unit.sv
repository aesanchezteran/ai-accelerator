`include "config.vh"
`include "structs.sv"
`include "enums.sv"

module control_unit(
    input logic                             clk,
    input logic                             rst,
    input data_union_in_accelerator_t       cu_instr_bus,
    control_to_mcst.master                  cu_to_mcst,
    output cu_act_sp_control_t              cu_to_act_control,
    output cu_ps_sp_control_t               cu_to_ps_control,
    output MUX_PS_selector_t                cu_to_mux_ps_control,
    output PE_selector_t                    pe_sys_selector,
    output logic                            instr_ready_signal
    );
    // Registered Input
    data_union_in_accelerator_t cu_instr_bus_r;
    logic [ACT_SP_ADDR_SIZE-1:0] addra_ps_r;
    logic [ACT_SP_ADDR_SIZE-1:0] addrb_ps_r;
    
    // States in FSM
    opcode_enum_t n_state;
    opcode_enum_t c_state;
    
    //Address difference
    logic [ACT_SP_ADDR_SIZE-1:0] addr_diff;
    logic [ACT_SP_ADDR_SIZE-1:0] cnt;
    
    logic [ACT_SP_ADDR_SIZE-1:0] addr_ps_mov;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            c_state <= IDLE_op;
            cnt <= '0;
            addr_ps_mov <= '0;
            addr_diff <= '0;
        end else begin
            c_state <= n_state;
            cu_instr_bus_r <= cu_instr_bus;
            
            addr_diff <= cu_instr_bus_r.matrix_multiply_fields.end_address_activation - cu_instr_bus_r.matrix_multiply_fields.start_address_activation;
            if (c_state == MM_op) begin
                cnt <= cnt + 1; 
            end else begin
                cnt <= '0; 
            end
            
            if (c_state == MM_op && cnt < ARRAY_HEIGHT) begin
                addr_ps_mov <= cu_instr_bus.matrix_multiply_fields.address_ps;
            end else begin
                addr_ps_mov <= addr_ps_mov + 1;
            end
            
            
        end
    end
    
    always_comb begin
        if (n_state == PSUM_ACC_op) begin
            addra_ps_r = cu_instr_bus.partial_sum_accumulate_fields.address_ps_1;
            addrb_ps_r = cu_instr_bus.partial_sum_accumulate_fields.address_ps_2;
        end else begin
            addra_ps_r = '0;
            addrb_ps_r = '0;
        end
    end
    
    always_comb begin
        case(c_state)
            MM_op: begin
                if (cnt < addr_diff + 1) begin 
                    cu_to_act_control.re = 1;
                end else begin
                    cu_to_act_control.re = 0;
                end
                if (cnt < ARRAY_HEIGHT || cnt > (addr_diff + ARRAY_HEIGHT)) begin
                    cu_to_ps_control.we_a = 0;
                end else begin
                    cu_to_ps_control.we_a = 1;
                end
            end
            PSUM_ACC_op: begin
                cu_to_ps_control.we_a = 1;
            end
            default: begin
                cu_to_ps_control.we_a = '0;
                cu_to_act_control.re = 0;
            end
        endcase
    end
    // CC1
    always_comb begin
        case(cu_instr_bus.opcode_fields.opcode)
            3'b000: n_state = IDLE_op;
            3'b001: n_state = WEIGHT_STORE_op;
            3'b010: n_state = ACT_STORE_op;
            3'b011: n_state = PSUM_STORE_op;
            3'b100: n_state = MM_op;
            3'b101: n_state = PSUM_ACC_op;
            3'b110: n_state = PS_COLL_op;
            default: n_state = IDLE_op;
        endcase
    end
    
    // CC2
    always_comb begin
        case(c_state)
            IDLE_op: begin
                cu_to_act_control.we = '0;
                cu_to_act_control.address = '0;
                cu_to_act_control.data = '0;
                
                cu_to_ps_control.we_b = '0;
                cu_to_ps_control.re_a = '0;
                cu_to_ps_control.re_b = '0;
                cu_to_ps_control.address_a = '0;
                cu_to_ps_control.address_b = '0;
                cu_to_ps_control.data = '0;
                
                cu_to_mcst.data = '0;
                cu_to_mcst.enable = '0;
                cu_to_mcst.id = '0;
                
                cu_to_mux_ps_control = MUX_SYSTOLIC;
                
                pe_sys_selector = PE_IDLE;
                
                instr_ready_signal = 1;
            end
            WEIGHT_STORE_op: begin
                cu_to_act_control.we = '0;
                cu_to_act_control.address = '0;
                cu_to_act_control.data = '0;

                cu_to_ps_control.we_b = '0;
                cu_to_ps_control.re_a = '0;
                cu_to_ps_control.re_b = '0;
                cu_to_ps_control.address_a = '0;
                cu_to_ps_control.address_b = '0;
                cu_to_ps_control.data = '0;
             
                cu_to_mcst.data = cu_instr_bus_r.weight_store_fields.data;
                cu_to_mcst.id = cu_instr_bus_r.weight_store_fields.row_id;
                if (&cu_to_mcst.ready) begin
                    cu_to_mcst.enable = 1;
                end else begin
                    cu_to_mcst.enable = 0;
                end
                
                cu_to_mux_ps_control = MUX_SYSTOLIC;
                
                pe_sys_selector = PE_WEIGHT_FETCH;
                
                instr_ready_signal = 1;
            end
            ACT_STORE_op: begin
                cu_to_act_control.we = 1;
                cu_to_act_control.address = cu_instr_bus_r.activation_store_fields.address;
                cu_to_act_control.data = cu_instr_bus_r.activation_store_fields.data;
      
                cu_to_ps_control.we_b = '0;
                cu_to_ps_control.re_a = '0;
                cu_to_ps_control.re_b = '0;
                cu_to_ps_control.address_a = '0;
                cu_to_ps_control.address_b = '0;
                cu_to_ps_control.data = '0;
                
                cu_to_mcst.data = '0;
                cu_to_mcst.enable = '0;
                cu_to_mcst.id = '0;
                
                cu_to_mux_ps_control = MUX_SYSTOLIC;
                
                pe_sys_selector = PE_IDLE;
                
                instr_ready_signal = 1;
            end
            
            MM_op: begin
                cu_to_act_control.we = 0;
                cu_to_act_control.address = cu_instr_bus_r.matrix_multiply_fields.start_address_activation + cnt;
                cu_to_act_control.data = '0;
        
                cu_to_ps_control.we_b = '0;
                cu_to_ps_control.re_a = '0;
                cu_to_ps_control.re_b = '0;
                
                cu_to_ps_control.data = '0;
                
                cu_to_mcst.data = '0;
                cu_to_mcst.enable = '0;
                cu_to_mcst.id = '0;
                
                cu_to_mux_ps_control = MUX_SYSTOLIC;
                
                pe_sys_selector = PE_MULTIPLY;
                
                if (cnt < (ARRAY_HEIGHT + ARRAY_WIDTH) + ARRAY_HEIGHT - 1) begin
                    instr_ready_signal = 0;
                    cu_to_ps_control.address_a = addr_ps_mov;
                    cu_to_ps_control.address_b = '0;
                end else begin
                    instr_ready_signal = 1;
                    cu_to_ps_control.address_a = addra_ps_r;
                    cu_to_ps_control.address_b = addrb_ps_r;
                    
                end
            end
            
            PSUM_ACC_op: begin
                cu_to_act_control.we = 0;
                cu_to_act_control.address = '0;
                cu_to_act_control.data = '0;
        
                cu_to_ps_control.we_b = '0;
                cu_to_ps_control.re_a = '1;
                cu_to_ps_control.re_b = '1;
                cu_to_ps_control.address_a = addra_ps_r;
                cu_to_ps_control.address_b = addrb_ps_r;
                cu_to_ps_control.data = '0;
                
                cu_to_mcst.data = '0;
                cu_to_mcst.enable = '0;
                cu_to_mcst.id = '0;
                
                cu_to_mux_ps_control = MUX_ALUS;
                
                pe_sys_selector = PE_IDLE;
                
                instr_ready_signal = 1;
            end
            
            PS_COLL_op: begin
                cu_to_act_control.we = 0;
                cu_to_act_control.address = '0;
                cu_to_act_control.data = '0;
        
                cu_to_ps_control.we_b = '0;
                cu_to_ps_control.re_a = '1;
                cu_to_ps_control.re_b = '0;
                cu_to_ps_control.address_a = cu_instr_bus_r.partial_sum_collect_fields.address_ps;
                cu_to_ps_control.address_b = '0;
                cu_to_ps_control.data = '0;
                
                cu_to_mcst.data = '0;
                cu_to_mcst.enable = '0;
                cu_to_mcst.id = '0;
                
                cu_to_mux_ps_control = MUX_SYSTOLIC;
                
                pe_sys_selector = PE_IDLE;
                
                instr_ready_signal = 1;
            end
            
            
            default: begin
                cu_to_act_control.we = '0;
                cu_to_act_control.address = '0;
                cu_to_act_control.data = '0;

                cu_to_ps_control.we_b = '0;
                cu_to_ps_control.re_a = '0;
                cu_to_ps_control.re_b = '0;
                cu_to_ps_control.address_a = '0;
                cu_to_ps_control.address_b = '0;
                cu_to_ps_control.data = '0;
                
                cu_to_mcst.data = '0;
                cu_to_mcst.enable = '0;
                cu_to_mcst.id = '0;
                
                cu_to_mux_ps_control = MUX_SYSTOLIC;
                
                pe_sys_selector = PE_IDLE;
                instr_ready_signal = 1;
            end
        endcase
    end
    
    
endmodule
