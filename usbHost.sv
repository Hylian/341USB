// Write your usb host here.  Do not modify the port list.

typedef enum logic [1:0] {bus_J = 1, bus_K = 0, bus_SE0 = 2} busState;

module usbHost
    (input logic clk, rst_L,
    usbWires wires);
   
    /* Tasks needed to be finished to run testbenches */
    logic [87:0] packet, dnc_packet; //This accounts for the largest packet size
  
    busState enc_busState, bs_outputBusState, nrzi_outputBusState;
    busState dnc_busState, nrdec_outputBusState, nrdec_inputBusState;
   
    logic enc_dataReady, enc_okToSend;
    logic bs_dataReady, bs_stuffEnable;
    logic nrzi_dataReady, nrzi_outputValid;
  
    logic nrdec_dataReady;

    //The following is for testing
    assign nrdec_dataReady = ~nrzi_outputValid;
    //assign nrdec_inputBusState = nrzi_outputBusState;

    assign wires.DP = nrzi_outputValid ? (nrzi_outputBusState == bus_J) : 1'bz;
    assign wires.DM = nrzi_outputValid ? (nrzi_outputBusState == bus_K) : 1'bz;

    assign wires.DP = nrzi_outputValid ? 1'bz : (nrdec_inputBusState == bus_J);
    assign wires.DM = nrzi_outputValid ? 1'bz : (nrdec_inputBusState == bus_K);

    task prelabRequest();
    // sends an OUT packet with ADDR=5 and ENDP=4
     
    packet <= {4'd4,7'd5,~4'd1,4'd1};
    enc_dataReady <= 0;
    #10 @(posedge clk) enc_dataReady <= 1;
    @(posedge clk) enc_dataReady <= 0;
    repeat(100)@(posedge clk) packet = 0;

    endtask: prelabRequest

    // host sends memPage to thumb drive and then gets data back from it
    // then returns data and status to the caller
    task readData
    (input  bit [15:0] mempage, // Page to write
     output bit [63:0] data, // array of bytes to write
     output bit        success);

    endtask: readData

    // Host sends memPage to thumb drive and then sends data
    // then returns status to the caller
    task writeData
    (input  bit [15:0] mempage, // Page to write
     input  bit [63:0] data, // array of bytes to write
     output bit        success);

    endtask: writeData

    encoder enc0(clk, rst_L, enc_dataReady, enc_okToSend, packet, 
                 enc_busState, bs_dataReady, bs_stuffEnable);

    bitStuff bs0(clk, rst_L, bs_dataReady, bs_stuffEnable, enc_busState, 
                 enc_okToSend, nrzi_dataReady, bs_outputBusState);

    nrziEncode nrzi0(clk, rst_L, nrzi_dataReady, bs_outputBusState, 
                     nrzi_outputValid, nrzi_outputBusState);
  
    decoder dnc0(clk, rst_L, bu_outputReady, dnc_packet, dnc_busState, 
                 dnc_dataReady, bu_unstuffEnable);

    bitUnstuff bu0(clk, rst_L, nrdec_outputValid, bu_unstuffEnable, 
                   nrdec_outputBusState, bu_outputReady, dnc_busState);
   
    nrziDecode nrdec0(clk, rst_L, nrdec_dataReady, nrdec_inputBusState, 
                      nrdec_outputValid, nrdec_outputBusState);
   
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
                if(stuffEnable && inputBusState == bus_J) 
		    counter <= counter + 1;
                else counter <= 0;
            end
        end
    end

endmodule : bitStuff

module nrziEncode
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
                else if(inputBusState == bus_K) 
		    outputBusState <= (outputBusState == bus_K) ? bus_J : bus_K;
                else if(inputBusState == bus_SE0) outputBusState <= bus_SE0;
            end
            else outputValid <= 0;
        end
    end
endmodule : nrziEncode


module bitUnstuff
    (input logic clk, rst_L, dataReady, unstuffEnable,
     input busState inputBusState,
     output logic outputReady,
     output busState outputBusState);

    logic [2:0]	counter;

    always_comb begin
        //If unstuff is not enabled then this will act as a pass through
        outputReady = (dataReady && !(counter == 6)) || !unstuffEnable;
        outputBusState = inputBusState;   
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
                if(unstuffEnable && inputBusState == bus_J) 
		    counter <= counter + 1;
                else counter <= 0;
            end
        end
    end
endmodule : bitUnstuff

module nrziDecode
    (input logic clk, rst_L, dataReady,
     input busState inputBusState,
     output logic outputValid,
     output busState outputBusState);

    busState lastBusState;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            outputBusState <= bus_J;
            outputValid <= 0;
	    lastBusState <= bus_J;
	end
        else begin
            if(dataReady) begin
                outputValid <= 1;
                if(outputBusState == bus_SE0) outputBusState <= inputBusState;
                else if(inputBusState == bus_SE0) outputBusState <= bus_SE0;
                else if(inputBusState == lastBusState) outputBusState <=  bus_J;
                else if(inputBusState != lastBusState) outputBusState <=  bus_K;
	        lastBusState <= inputBusState;
	       
            end
            else outputValid <= 0;
        end
    end
endmodule : nrziDecode

module decoder
    (input logic clk, rst_L, dataReady,
     output logic [87:0] packet,
     input busState inputBusState,
     output logic outputReady, unstuffEnable);

    logic [7:0] index; //Index counter
    logic [87:0] outputReg; //Holds our data
    logic [7:0] counter;
	   
    enum {WAIT, SYNC, PID, DATA, CRC, EOP} state, nextState;
    typedef enum logic[3:0] {OUT = 4'b0001, IN = 4'b1001, DATA0 = 4'b0011,
                     ACK = 4'b0010, NAK = 4'b1010} pidValue;
    pidValue pid;

    //First we wait to see if we should recieve data by monitoring the
    //inputBusState for the Sync
   
    assign packet = outputReg;
      
    always_comb begin
        pid = pidValue'(outputReg[3:0]); 
        unstuffEnable = 0;
        case(state)
            // WAIT: Stall until we are able to start
            WAIT: begin
                nextState = (dataReady) ? SYNC : WAIT;
            end
            // SYNC: recieve the the sync byte (00000001)
            SYNC: begin
                nextState = (counter == 8'd7) ? DATA : SYNC;
            end
            // DATA: Increment counter to recieve the data into packet
            DATA: begin
                unstuffEnable = index >= 8;
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
		    default: begin
		        nextState = DATA;
		    end
                endcase
            end
            // CRC: Takes in the CRC of the payload 
            CRC: begin
                unstuffEnable = 1;
	        nextState = (counter == 0) ? EOP : CRC;
            end
            // EOP: Send EOP signal
            EOP: begin
                if(counter == 8'd2) begin
                    nextState = WAIT;
                end
                else begin
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
            outputReady <= 0;
	    outputReg <= 0;
        end
        else begin
            if(state == WAIT) begin
                counter <= 0;
                index <= 0;
            end
            if(state == SYNC && dataReady) begin
	        //First we see if we have recieved the sync
                if(nextState == DATA) begin
                    counter <= 0;
		    outputReg <= 0;
		    outputReady <= 0; 
		    //Currently the output is still the old packet 
		    // which is valid until the sync
		end
	        //Then we wait until we see a stream of at least 7 zeros
	        else if(inputBusState == bus_K && counter != 8'd7) begin
		    counter <= counter + 1;
		end
	        //Then we look for a 1
	        else if(inputBusState == bus_J && counter == 8'd7) begin
		    counter <= counter + 1;
		end
	        //If we get a zero before this, we reset counter
	        else if(counter != 8'd7) begin
		    counter <= 0;
		end
            end
            if(state == DATA && dataReady) begin
	        //We need to assign data to the inputReg 
	        // by converting the state into a 1 or 0
	        if(inputBusState == bus_J) begin
		    outputReg[index] <= 1;
		end
		else begin
		    outputReg[index] <= 0;
		end
	        index <= index + 1;
	        
                if(nextState == CRC) begin
                    if(pid == DATA0) begin
		        counter <= 15;
		        index <= index + 16;
		    end
                    else begin
		        counter <= 4;
		        index <= index + 5;
		    end
                end
            end
            if(state == CRC) begin
	        //We need to assign data to the inputReg 
	        // by converting the state into a 1 or 0
	        //However, the crc is inverted and in MSB 
	        // so we need to go from left to right
	        //and invert it
	        if(inputBusState == bus_J) begin
		    outputReg[index - counter] <= 0;
		end
		else begin
		    outputReg[index - counter] <= 1;
		end
                counter <= counter - 1;
            end
            if(nextState == EOP) begin
                counter <= 0;
            end
            if(state == EOP) begin
                counter <= counter + 1;
	        if(nextState == WAIT) begin
		    outputReady <= 1;
		end
            end
            state <= nextState;
        end
    end
   
endmodule : decoder
