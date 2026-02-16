module breadboard_led_traffic_controller(
    input        CLOCK_50,  // PIN_AF14
    input        UART_RXD,  // PIN_AJ27
    output       UART_TXD,  // PIN_AK29
    input  [0:0] KEY,       // PIN_AA14
    output [11:0] GPIO_1,
    output [6:0]  HEX0
);

    // --- NEW STATES FOR 1-AT-A-TIME SEQUENCE ---
    parameter S_N_G = 4'd0, S_N_Y = 4'd1;
    parameter S_S_G = 4'd2, S_S_Y = 4'd3;
    parameter S_E_G = 4'd4, S_E_Y = 4'd5;
    parameter S_W_G = 4'd6, S_W_Y = 4'd7;
    
    // --- EMERGENCY STATES ---
    parameter E_TRANS = 4'd8; 
    parameter E_NORTH = 4'd9, E_SOUTH = 4'd10, E_EAST = 4'd11, E_WEST = 4'd12;

    reg [3:0] state, timer;
    reg [25:0] clk_count;
    wire tick = (clk_count == 49_999_999); // 1-second tick at 50MHz
    
    wire [7:0] rx_data;
    wire rx_done, tx_busy;
    reg [7:0] tx_data, active_emergency, prev_emergency;
    reg tx_start;
    reg [1:0] tx_step; 

    // --- Instantiations ---
    uart_rx receiver (.clk(CLOCK_50), .rx_line(UART_RXD), .data_out(rx_data), .done(rx_done));
    uart_tx transmitter (.clk(CLOCK_50), .data_in(tx_data), .start(tx_start), .tx_line(UART_TXD), .busy(tx_busy));
    hex_decoder h0 (.bin(timer), .seg(HEX0));

    always @(posedge CLOCK_50 or negedge KEY[0]) begin
        if (!KEY[0]) begin
            // Reset state triggered by board push button
            state <= S_N_G; timer <= 4'd9; clk_count <= 0; 
            active_emergency <= 8'h43; prev_emergency <= 8'h43; tx_step <= 0;
        end else begin
            clk_count <= (tick) ? 0 : clk_count + 1;
            
            // --- TELEMETRY ---
            // Sends current status back over UART
            if (tick && !tx_busy && tx_step == 0) begin
                tx_data <= (active_emergency == 8'h43) ? 8'h43 : active_emergency;
                tx_start <= 1; tx_step <= 1;
            end else if (!tx_busy && tx_step == 1) begin
                tx_data <= 8'h30 + timer; tx_start <= 1; tx_step <= 0;
            end else tx_start <= 0;

            // --- LATCH UART DATA ---
            if (rx_done) begin
                active_emergency <= rx_data;
                prev_emergency <= rx_data;
            end

            // --- UNIFIED FSM (Emergency has strict priority over normal tick) ---
            if (rx_done && (rx_data != prev_emergency) && (rx_data != 8'h43) && (state < E_TRANS)) begin
                // TOP PRIORITY: Jump to transition state when valid new emergency arrives
                state <= E_TRANS; 
                timer <= 4'd3; 
            end else if (tick) begin
                if (timer > 0) begin
                    timer <= timer - 1;
                end else begin
                    case (state)
                        // Normal 1-at-a-time Sequence: N -> S -> E -> W
                        S_N_G:   begin state <= S_N_Y; timer <= 4'd3; end
                        S_N_Y:   begin state <= S_S_G; timer <= 4'd9; end
                        S_S_G:   begin state <= S_S_Y; timer <= 4'd3; end
                        S_S_Y:   begin state <= S_E_G; timer <= 4'd9; end
                        S_E_G:   begin state <= S_E_Y; timer <= 4'd3; end
                        S_E_Y:   begin state <= S_W_G; timer <= 4'd9; end
                        S_W_G:   begin state <= S_W_Y; timer <= 4'd3; end
                        S_W_Y:   begin state <= S_N_G; timer <= 4'd9; end
                        
                        // Emergency Transitions
                        E_TRANS: begin
                            case(active_emergency)
                                8'h4E: state <= E_NORTH; // 'N'
                                8'h53: state <= E_SOUTH; // 'S'
                                8'h45: state <= E_EAST;  // 'E'
                                8'h57: state <= E_WEST;  // 'W'
                                default: state <= S_N_G; // Failsafe
                            endcase
                            timer <= 4'd9;
                        end
                        E_NORTH, E_SOUTH, E_EAST, E_WEST: begin
                            // 'C' (Clear) returns to North Green
                            if (active_emergency == 8'h43) begin 
                                state <= S_N_G; timer <= 4'd9; 
                            end else begin
                                timer <= 4'd9; // Hold green indefinitely
                            end
                        end
                        default: begin state <= S_N_G; timer <= 4'd9; end
                    endcase
                end
            end
        end
    end

    // --- LED Logic (3 sides red while 1 is green) ---
    // North (Pins 0, 1, 2)
    assign GPIO_1[0] = (state == S_N_G || state == E_NORTH); // Green
    assign GPIO_1[1] = (state == S_N_Y || state == E_TRANS); // Yellow
    assign GPIO_1[2] = !(GPIO_1[0] || GPIO_1[1]);            // Red
    
    // South (Pins 3, 4, 5)
    assign GPIO_1[3] = (state == S_S_G || state == E_SOUTH); // Green
    assign GPIO_1[4] = (state == S_S_Y || state == E_TRANS); // Yellow
    assign GPIO_1[5] = !(GPIO_1[3] || GPIO_1[4]);            // Red
    
    // East (Pins 6, 7, 8)
    assign GPIO_1[6] = (state == S_E_G || state == E_EAST);  // Green
    assign GPIO_1[7] = (state == S_E_Y || state == E_TRANS); // Yellow
    assign GPIO_1[8] = !(GPIO_1[6] || GPIO_1[7]);            // Red
    
    // West (Pins 9, 10, 11)
    assign GPIO_1[9] = (state == S_W_G || state == E_WEST);  // Green
    assign GPIO_1[10]= (state == S_W_Y || state == E_TRANS); // Yellow
    assign GPIO_1[11]= !(GPIO_1[9] || GPIO_1[10]);           // Red

endmodule

// ==========================================================
// SUB-MODULES
// ==========================================================

module uart_tx (input clk, input [7:0] data_in, input start, output reg tx_line, output reg busy);
    parameter CLKS_PER_BIT = 434; // 115200 baud at 50MHz
    reg [2:0] state = 0; reg [9:0] clk_count = 0; reg [2:0] bit_idx = 0; reg [7:0] tx_data = 0;
    
    always @(posedge clk) begin
        case (state)
            0: begin tx_line <= 1; busy <= 0; if (start) begin tx_data <= data_in; state <= 1; busy <= 1; end end
            1: begin tx_line <= 0; if (clk_count < CLKS_PER_BIT-1) clk_count <= clk_count + 1; else begin clk_count <= 0; state <= 2; end end
            2: begin tx_line <= tx_data[bit_idx]; if (clk_count < CLKS_PER_BIT-1) clk_count <= clk_count + 1; else begin clk_count <= 0; if (bit_idx < 7) bit_idx <= bit_idx + 1; else state <= 3; end end
            3: begin tx_line <= 1; if (clk_count < CLKS_PER_BIT-1) clk_count <= clk_count + 1; else begin state <= 0; end end
        endcase
    end
endmodule

module uart_rx (input clk, input rx_line, output reg [7:0] data_out, output reg done);
    parameter CLKS_PER_BIT = 434; // 115200 baud at 50MHz
    reg [2:0] state = 0; reg [9:0] clk_count = 0; reg [2:0] bit_idx = 0;
    
    always @(posedge clk) begin
        case (state)
            0: begin done <= 0; clk_count <= 0; bit_idx <= 0; if (rx_line == 0) state <= 1; end
            1: begin if (clk_count == (CLKS_PER_BIT-1)/2) begin if (rx_line == 0) begin clk_count <= 0; state <= 2; end else state <= 0; end else clk_count <= clk_count + 1; end
            2: begin if (clk_count < CLKS_PER_BIT-1) clk_count <= clk_count + 1; else begin clk_count <= 0; data_out[bit_idx] <= rx_line; if (bit_idx < 7) bit_idx <= bit_idx + 1; else state <= 3; end end
            3: begin if (clk_count < CLKS_PER_BIT-1) clk_count <= clk_count + 1; else begin done <= 1; state <= 0; end end
        endcase
    end
endmodule

module hex_decoder(input [3:0] bin, output reg [6:0] seg);
    // 7-segment display logic for active-low DE1-SoC displays
    always @(*) begin
        case(bin)
            4'h0: seg = 7'b1000000; 4'h1: seg = 7'b1111001; 4'h2: seg = 7'b0100100;
            4'h3: seg = 7'b0110000; 4'h4: seg = 7'b0011001; 4'h5: seg = 7'b0010010;
            4'h6: seg = 7'b0000010; 4'h7: seg = 7'b1111000; 4'h8: seg = 7'b0000000;
            4'h9: seg = 7'b0010000; default: seg = 7'b1111111;
        endcase
    end
endmodule