// Write your usb host here.  Do not modify the port list.

typedef enum logic [1:0] {bus_J = 1, bus_K = 0, bus_SE0 = 2} busState;

module usbHost
  (input logic clk, rst_L,
   usbWires wires);
   
  /* Tasks needed to be finished to run testbenches */
  logic [87:0] packet;

  busState enc_busState, bs_outputBusState, nrzi_outputBusState;

  logic enc_dataReady, enc_okToSend;
  logic bs_dataReady, bs_stuffEnable;
  logic nrzi_dataReady, nrzi_outputValid;


  assign wires.DP = nrzi_outputValid ? (nrzi_outputBusState == bus_J) : 1'bz;
  assign wires.DM = nrzi_outputValid ? (nrzi_outputBusState == bus_K) : 1'bz;

  task prelabRequest();
  // sends an OUT packet with ADDR=5 and ENDP=4
  // packet should have SYNC and EOP too
    //$monitor("clk(%x) enc.state(%s) enc.index(%d) enc.pid(%s) enc_busState(%s) bs_dataReady(%x) enc_okToSend(%x) nrzi_outputBusState(%s)", clk, enc0.state, enc0.index,  enc0.pid, enc_busState, bs_dataReady, enc_okToSend, nrzi_outputBusState);
    //$monitor("clk(%x) bs.counter(%d) bs_dataReady(%x) enc_okToSend(%x) nrzi_dataReady(%x) enc_busState(%x) bs_outputBusState(%x) nrzi_outputBusState(%s)", clk, bs0.counter, bs_dataReady, enc_okToSend, nrzi_dataReady, enc_busState, bs_outputBusState, nrzi_outputBusState);
    //$monitor("clk(%x) nrzi_dataReady(%x) nrzi_outputBusState(%s) nrzi_outputValid(%x) DP(%x) DM(%x)", clk, nrzi_dataReady, nrzi_outputBusState, nrzi_outputValid, wires.DP, wires.DM);
    //$monitor("crc5.state(%s) crc.counter(%d) crc5.dataReg(%b) crc5.crc(%b) crc5.crc0_next(%b) crc5.crc2_next(%b)", enc0.a1.state, enc0.a1.counter, enc0.a1.dataReg, enc0.a1.crc, enc0.a1.crc0_next, enc0.a1.crc2_next);
    packet <= {4'd4,7'd5,~4'd1,4'd1};
    enc_dataReady <= 0;
    #10 @(posedge clk) enc_dataReady <= 1;
    @(posedge clk) enc_dataReady <= 0;
    repeat(100)@(posedge clk) packet = 0;

  endtask: prelabRequest

  task readData
  // host sends memPage to thumb drive and then gets data back from it
  // then returns data and status to the caller
  (input  bit [15:0] mempage, // Page to write
   output bit [63:0] data, // array of bytes to write
   output bit        success);

  endtask: readData

  task writeData
  // Host sends memPage to thumb drive and then sends data
  // then returns status to the caller
  (input  bit [15:0] mempage, // Page to write
   input  bit [63:0] data, // array of bytes to write
   output bit        success);

  endtask: writeData

  encoder enc0(clk, rst_L, enc_dataReady, enc_okToSend, packet, enc_busState, bs_dataReady, bs_stuffEnable);

  bitStuff bs0(clk, rst_L, bs_dataReady, bs_stuffEnable, enc_busState, enc_okToSend, nrzi_dataReady, bs_outputBusState);

  nrzi nrzi0(clk, rst_L, nrzi_dataReady, bs_outputBusState, nrzi_outputValid, nrzi_outputBusState);

endmodule: usbHost

/*
 The encoder module is used to take a packet, look at its PID, and based on
 that PID it will send either the first 24 bits or all 88 bits. It will also
 first send the sync bits and will end with the EOP bits.
 */
module encoder
    (input logic clk, rst_L, dataReady, okToSend,
     input logic [87:0] packet,
     output busState outputBusState,
     output logic outputReady, stuffEnable);

    logic [7:0] index; //Index counter
    logic [87:0] inputReg; //Holds our data
    logic [15:0] crc, crc16Result;
    logic [7:0] counter;
    logic [4:0] crc5Result;
	   
    enum {WAIT, SYNC, DATA, CRC, EOP} state, nextState;
    typedef enum logic[3:0] {OUT = 4'b0001, IN = 4'b1001, DATA0 = 4'b0011,
                     ACK = 4'b0010, NAK = 4'b1010} pidValue;
    pidValue pid;

    crc5 a1(clk, rst_L, packet[18:8], dataReady, crc5Result);
    crc16 a2(clk, rst_L, packet[71:8], dataReady, crc16Result);
   
    always_comb begin
        crc = (pid == DATA0) ? ~crc16Result : ~crc5Result;
        outputBusState = bus_J;
        outputReady = (state != WAIT);
        pid = pidValue'(inputReg[3:0]);
        stuffEnable = 0;
        case(state)
            // WAIT: Stall until we receive a packet
            WAIT: begin
                nextState = (dataReady) ? SYNC : WAIT;
            end
            // SYNC: Send the sync byte (00000001)
            SYNC: begin
                if(counter == 8'd7) begin
                    nextState = DATA;
                    outputBusState = bus_J;
                end
                else begin
                    nextState = SYNC;
                    outputBusState = bus_K;
                end
            end
            // DATA: Increment a counter to transmit the packet payload one bit at a time
            DATA: begin
                outputBusState = busState'(inputReg[index]);
                stuffEnable = index >= 8;
                nextState = DATA;
                case(pid)
                    OUT: begin
                        if(index == 8'd18) nextState = CRC;
                    end
                    IN: begin
                        if(index == 8'd18) nextState = CRC;
                    end
                    DATA0: begin
                        if(index == 8'd71) nextState = CRC;
                    end
                    ACK: begin
                        if(index == 8'd7) nextState = EOP;
                    end
                    NAK: begin
                        if(index == 8'd7) nextState = EOP;
                    end
                endcase
            end
            // CRC: Output the CRC of the payload 
            CRC: begin
                stuffEnable = 1;
                outputBusState = busState'(crc[counter]);
                if(counter == 0) begin
                    nextState = EOP;
                end
                else nextState = CRC;
            end
            // EOP: Send EOP signal
            EOP: begin
                if(counter == 8'd2) begin
                    outputBusState = bus_J;
                    nextState = WAIT;
                end
                else begin
                    outputBusState = bus_SE0;
                    nextState = EOP;
                end
            end
        endcase
    end

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            state <= WAIT;
            counter <= 0;
            index <= 0;
        end
        else begin
            if(state == WAIT) begin
                counter <= 0;
                index <= 0;
            end
            if(state == WAIT && nextState == SYNC) begin
                inputReg <= packet;
            end
            if(state == SYNC) begin
                counter <= counter + 1;
                if(nextState == DATA) begin
                    counter <= 0;
                end
            end
            if(state == DATA) begin
                index <= index + 1;
                if(nextState == CRC) begin
                    if(pid == DATA0) counter = 15;
                    else counter = 4;
                end
            end
            if(state == CRC) begin
                counter <= counter - 1;
            end
            if(nextState == EOP) begin
                counter <= 0;
            end
            if(state == EOP) begin
                counter <= counter + 1;
            end
            state <= nextState;
        end
    end
   
endmodule : encoder

module crc5
    (input logic clk, rst_L,
     input logic [10:0] data,
     input logic dataReady,
     output logic [4:0] crc);

    enum {WAIT, GO} state, nextState;

    logic [10:0] dataReg;
    logic [4:0] counter;

    logic crc0_next, crc2_next;

    always_comb begin
        case(state)
            WAIT: nextState = (dataReady) ? GO : WAIT;
            GO: nextState = (counter == 5'd10) ? WAIT : GO;
        endcase

        crc0_next = dataReg[counter]^crc[4];
        crc2_next = (crc0_next^crc[1]);
    end
   
    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            state <= WAIT;
            crc <= 5'h1f;
            counter <= 0;
        end
        else begin
            if(state == WAIT && nextState == GO) begin
                dataReg <= data;
                crc <= 5'h1f;
                counter <= 0;
            end
            if(state == GO) begin
                crc[0] <= crc0_next;
                crc[1] <= crc[0];
                crc[2] <= crc2_next;
                crc[4:3] <= crc[3:2];
                counter <= counter + 1;
            end
            state <= nextState;
        end
    end

endmodule : crc5

module crc16
    (input logic clk, rst_L,
     input logic [63:0]  data,
     input logic dataReady,
     output logic [15:0] crc);

   
    enum {WAIT, GO} state, nextState;

    logic [63:0] dataReg;
    logic [7:0] counter;

    always_comb begin
        case(state)
            WAIT: nextState = (dataReady) ? GO : WAIT;
            GO: nextState = (counter == 8'd63) ? WAIT : GO;
        endcase
    end

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            state <= WAIT;
            crc <= 0;
            counter <= 0;
        end
        else begin
            if(state == WAIT && nextState == GO) begin
                dataReg <= data;
                crc <= 0;
                counter <= 0;
            end
            if(state == GO) begin
                crc[0] <= (dataReg[counter]^crc[15]);
                crc[1] <= crc[0];
                crc[2] <= (dataReg[counter]^crc[15])^crc[1];
                crc[14:3] <= crc[13:2];
                crc[15] <= (dataReg[counter]^crc[15])^crc[14];
                counter <= counter + 1;
            end
            state <= nextState;
        end
    end

endmodule : crc16

module bitStuff
    (input logic clk, rst_L, dataReady, stuffEnable,
     input busState inputBusState,
     output logic readyToReceive, outputReady,
     output busState outputBusState);

    logic [2:0]	counter;

    always_comb begin
        outputReady = dataReady || (counter == 6);
        outputBusState = (counter == 6) ? bus_K : inputBusState;   
        readyToReceive = !(counter == 5);
    end

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            counter <= 0;
        end   
        else begin
            if(counter == 6) begin
                counter <= 0;
            end
            else if(dataReady) begin
                if(stuffEnable && inputBusState == bus_J) counter <= counter + 1;
                else counter <= 0;
            end
        end
    end

endmodule : bitStuff

module nrzi
    (input logic clk, rst_L, dataReady,
     input busState inputBusState,
     output logic outputValid,
     output busState outputBusState);

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            outputBusState <= bus_J;
            outputValid <= 0;
        end
        else begin
            if(dataReady) begin
                outputValid <= 1;
                if(outputBusState == bus_SE0) outputBusState <= inputBusState;
                else if(inputBusState == bus_K) outputBusState <= (outputBusState == bus_K) ? bus_J : bus_K;
                else if(inputBusState == bus_SE0) outputBusState <= bus_SE0;
            end
            else outputValid <= 0;
        end
    end
endmodule : nrzi

