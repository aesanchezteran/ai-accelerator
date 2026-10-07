
module rams_sp_rf_rst #(
    parameter       WIDTH   = 16,
    parameter       DEPTH   = 4096
)(
    input logic                     clk,
    input logic                     en,
    input logic                     we,
    
    input logic [$clog2(DEPTH)-1:0] addr,
    input logic [WIDTH-1:0]         di,
    output logic [WIDTH-1:0]        dout
    );
    
    logic [WIDTH-1:0] ram [DEPTH-1:0];
    
    always_ff @(posedge clk) begin
        if (en) begin
            if (we) begin
                ram[addr] <= di;
            end
            
        end
    end
    assign dout = ram[addr];
    
endmodule
