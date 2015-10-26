// Write your usb host here.  Do not modify the port list.
module usbHost
  (input logic clk, rst_L,
   usbWires wires);
   
  /* Tasks needed to be finished to run testbenches */

  task prelabRequest();
  // sends an OUT packet with ADDR=5 and ENDP=4
  // packet should have SYNC and EOP too

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

  enum {J = 1, K = 0, SE0 = 2} enc_busState, bs_outputBusState, nrzi_outputBusState;

  logic enc_dataReady, enc_okToSend;
  encoder enc0(clk, rst_L, enc_dataReady, enc_okToSend, packet, enc_busState, bs_dataReady);

  logic bs_dataReady, bs_okToSend;
  bitStuff bs0(clk, rst_L, bs_dataReady, bs_okToSend, enc_busState, enc_okToSend, nrzi_dataReady, bs_outputBusState);

  logic nrzi_dataReady;
  nrzi nrzi0(clk, rst_L, nrzi_dataReady, bs_outputBusState, nrzi_outputBusState);

endmodule: usbHost

/*
 The encoder module is used to take a packet, look at its PID, and based on
 that PID it will send either the first 24 bits or all 88 bits. It will also
 first send the sync bits and will end with the EOP bits.
 */
module encoder
  (input logic clk, rst_L, dataReady, okToSend,
   input logic [87:0] packet,
   output busState,
   output logic outputReady);

   logic [7:0] 	      index; //Index counter
   logic [87:0]       inputReg; //Holds our data
   logic [15:0]       crc, crc16Result;
   logic [7:0] 	      count;
   logic [4:0] 	      crc5Result;
   
	   
   enum 	      {WAIT, SYNC, DATA, CRC, EOP} state, nextState;
   enum {OUT = 4'b0001, IN = 4'b1001, DATA0 = 4'b0011,
         ACK = 4'b0010, NAK = 4'b1010} pid;
   enum {J = 1, K = 0, SE0 = 2} busState;
   

   crc5 a1(clk, rst_L, packet[18:8], dataReady, crc5Result);
   crc16 a2(clk, rst_L, packet[71:8], dataReady, crc16Result);
   
   
   
   //Comb Logic
   always_comb begin
      crc = (pid == DATA0) ? crc16Result : crc5Result;
      busState = J;
      pid = inputReg[3:0];
      case(state)
	WAIT: begin
	   nextState = (dataReady) ? SYNC : WAIT;
	end
	SYNC: begin //Sends a 7 zeros then a 1 to sync the clock
	   if(counter == 8'd7) begin
	      nextState = DATA;
	   end
	   else begin
	      nextState = SYNC;
	      busState = K;
	   end
	end
	DATA: begin
	   busState = inputReg[index];
	   nextState = DATA;
	   case(pid)
	     OUT: begin
		if(index == 8'd18) begin
		   nextState = CRC;
		end
	     end
	     IN: begin
		if(index == 8'd18) begin
		   nextState = CRC;
		end
	     end
	     DATA0: begin
		if(index == 8'd71) begin
		   nextState = CRC;
		end
	     end
	     ACK: begin
		if(index == 8'd7) begin
		   nextState = EOP;
		end
	     end
	     NAK: begin
		if(index == 8'd7) begin
		   nextState = EOP;
		end
	     end
	   endcase // case (pid)
	end // case: DATA
	CRC: begin
	   nextState = CRC;
	   busState = crcResult[counter];
	   if((pid == OUT || pid == IN) && (counter == 8'd4)) begin
	      nextState = EOP;
	   end
	   else if(pid == DATA0 && counter == 8'd15) begin
	      nextState = EOP;
	   end
	end
	EOP: begin
	   if(counter = 8'd2) begin
	      busState = J;
	      nextState = WAIT;
	   end
	   else begin
	      busState = SE0;
	      nextState = EOP;
	   end
	end // case: EOP
      endcase // case (state)
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
	 end
	 if(state == CRC) begin
	    counter <= counter + 1;
	    if(nextState == EOP) begin
	       counter <= 0;
	    end
	 end
	 if(state == EOP && nextState == WAIT) begin
	    counter <= 0;
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

    always_comb begin
        case(state)
            WAIT: nextState = (dataReady) ? GO : WAIT;
            GO: nextState = (counter == 5'd10) ? WAIT : GO;
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
                crc[0] <= (dataReg[counter]^crc[4]);
                crc[1] <= crc[0];
                crc[2] <= (dataReg[counter]^crc[4])^crc[1];
                crc[3] <= crc[2];
                crc[4] <= crc[3];
                counter <= counter + 1;
            end
            state <= nextState;
        end
    end

endmodule : crc5

module crc16
    (input logic clk, rst_L,
     input logic [71:0]  data,
     input logic dataReady,
     output logic [15:0] crc);

   
    enum {WAIT, GO} state, nextState;

    logic [71:0] dataReg;
    logic [7:0] counter;

    always_comb begin
        case(state)
            WAIT: nextState = (dataReady) ? GO : WAIT;
            GO: nextState = (counter == 8'd71) ? WAIT : GO;
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
    (input logic clk, rst_L, dataReady, okToSend,
     input inputBusState,
     output logic readyToReceive, outputReady,
     output outputBusState);

    enum {J = 1, K = 0, SE0 = 2} inputBusState, outputBusState;
   
    logic [2:0]	counter;

    always_comb begin
        outputBusState = (counter == 6) ? K : inputBusState;   
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
                if(inputBusState == J) counter <= counter + 1;
                else counter <= 0;
            end
        end
    end

endmodule : bitStuff

module nrzi
    (input logic clk, rst_L, dataReady,
     input inputBusState,
     output outputBusState);

    enum {J = 1, K = 0, SE0 = 2} inputBusState, outputBusState;

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            outputBusState <= J;
        end
        else begin
            if(dataReady) begin
                if(inputBusState == K) outputBusState <= (outputBusState == K) ? J : K;
                else if(inputBusState == SE0) outputBusState <= SE0;
                else outputBusState <= inputBusState;
            end
        end
    end
endmodule : nrzi

