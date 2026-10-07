module interrupt_controller(
    input logic clk,
    input logic rst,
    input logic enable,
    input logic instruction_done,
    input logic axi_irq_clear,
    input logic axi_irq_enable,
    output logic interrupt
    );
    
    always_ff @(posedge clk) begin
        if (rst || ~enable) begin
            interrupt <= 1'b0;
        end else begin
            interrupt <= instruction_done;
        end 
    end     
endmodule
