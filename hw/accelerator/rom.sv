
module rom #(
    parameter initialize_path = "",
    parameter address_size = 3,
    parameter word_size = 64,
    parameter mem_height = 7
)(
    input logic [address_size-1:0] address,
    output logic [word_size-1:0] data
);
    logic [word_size-1:0] rom_memory [mem_height-1:0];
    initial begin
        $readmemh(initialize_path, rom_memory);
    end
    always_comb begin
        data = rom_memory[address];
    end
endmodule