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
    logic bs_dataReady, bs_stuffEnable, dnc_dataReady;
    logic nrzi_dataReady, nrzi_outputValid;
  
    logic nrdec_dataReady;

    logic start_t, txType_t, done_t, txType_p, start_p, done_p;
    logic [15:0] memAddrIn_t;
    logic [63:0] dataIn_t;
    logic [63:0] dataOut_t;
    logic [3:0] endpoint_p;
    logic [63:0] dataIn_p;
    logic [63:0] dataOut_p;
    logic error;
    logic temprst;

    assign nrdec_dataReady = !nrzi_outputValid;

    //Below assigns the output
    assign wires.DP = (nrzi_outputValid) ? (nrzi_outputBusState == bus_J) : 1'bz;
    assign wires.DM = (nrzi_outputValid) ? (nrzi_outputBusState == bus_K) : 1'bz;

    always_comb begin
        if(!wires.DP && !wires.DM) nrdec_inputBusState = bus_SE0;
        else if(wires.DP) nrdec_inputBusState = bus_J;
        else nrdec_inputBusState = bus_K;
    end

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
        //$monitor("enc0.packet(%b) \n dec.packet(%b) \n dncinput(%s)", enc0.packet, dnc0.packet, dnc0.inputBusState);
        //$display("Entering readData");
        //$monitor("rw0.state(%s) rw0.done_p(%b) p0.state(%s) p0.readyToReceive_rw(%b) p0.done_enc(%b) p0.done_dec(%b) enc0.state(%s) enc0.inputReg(%b) enc0.pid(%s) nrzi0.outputBusState(%s) nrzi0.outputValid(%b)", rw0.state, rw0.done_p, p0.state, p0.readyToReceive_rw, p0.done_enc, p0.done_dec, enc0.state, enc0.inputReg, enc0.pid, nrzi0.outputBusState, nrzi0.outputValid);
        //temprst <= 1;
        //@(posedge clk) temprst <= 0;
        //@(posedge clk) temprst <= 1;
       //$monitor("wires %b %b, nrzi out = %s, nrzioutputValid=%b, dec=%b, enc=%b state=%s", wires.DP, wires.DM, nrzi_outputBusState, nrzi_outputValid, p0.done_dec, p0.done_enc, nrdec0.inputBusState);
        //$monitor("dnc0.state(%s) dnc0.packet(%b) p0.state(%s) p0.crc(%b)", dnc0.state, dnc0.packet, p0.state, p0.crc);
        //$monitor("p0.a2.dataReg(%b) p0.crc16Result(%b)", p0.a2.dataReg, p0.crc16Result);
       //$monitor("dnc0.state(%s) p0.state(%s), encoder = %s, done = %b, dataOut_t=%h, datain=%h", dnc0.state, p0.state, enc0.state, rw0.done_p, p0.endpoint_p, rw0.dataIn_p);      
        //$monitor("dnc0.state(%s) p0.state(%s), encoder = %s, done = %b, dataOut_t=%h, datain=%h", dnc0.state, p0.state, enc0.state, rw0.done_p, p0.packetIn_dec, p0.lastPacketIn);
        //@(posedge clk);
        memAddrIn_t <= mempage;
        txType_t <= 1;
        start_t <= 1;
       //@(posedge clk);
        wait(done_t);
       //@(posedge clk);
        start_t <= 0;
        data <= dataOut_t;
        success <= !error;
        @(posedge clk);
    endtask: readData

    // Host sends memPage to thumb drive and then sends data
    // then returns status to the caller
    task writeData
    (input  bit [15:0] mempage, // Page to write
     input  bit [63:0] data, // array of bytes to write
     output bit        success);
//        $monitor("rw0.state(%s), p0.state(%s) enc0.state(%s) enc0.pid(%b) \n dnc0(%s) nrdec_inputBusState(%s) nrdec_outputBusState(%s) packet(%b), p0.errorCounter(%d) \n dnc(%b) \n dnc index=%d, dnc pid=%s", rw0.state, p0.state, enc0.state, enc0.pid, dnc0.state, nrdec_inputBusState, nrdec_outputBusState, packet, p0.errorCounter, dnc0.outputReg, dnc0.index, dnc0.pid);
        //$monitor("%b  enc0.packet(%b) \n dec.packet(%b) \n dncinput(%s)", clk, enc0.packet, dnc0.packet, dnc0.inputBusState);
        //$display("Entering writedata");
        //$monitor("clk(%d) enc0.packet(%b) enc0.state(%s) enc0.outputBusState(%s) enc0.counter(%d) enc0.crc(%b)", clk, enc0.packet, enc0.state, enc0.outputBusState, enc0.counter, enc0.crc);
        //$monitor("enc0.a2.state(%s) enc0.a2.crc(%b) enc0.crc(%b)", enc0.a2.state, enc0.a2.crc, enc0.crc);
        //$monitor("bs0.outputBusState(%s)", bs0.outputBusState);
        //$monitor("nrzi0.outputBusState(%s)", nrzi0.outputBusState);
        //$monitor("DP(%b) DM(%b) nrzi0.outputBusState(%s)", wires.DP, wires.DM, nrzi0.outputBusState);
        //$monitor("nrzi0.outputBusState(%s) dpanddm are 1 (%b)", nrzi0.outputBusState, (wires.DP==1 && wires.DM==1));
        //$monitor("clk(%b) enc.state(%s) nrzi.outputBusState(%s) enc.stuffEnable(%b) bs0.stuffEnableLatch(%b) bs0.counter(%d)", clk, enc0.state, nrzi0.outputBusState, enc0.stuffEnable, bs0.stuffEnableLatch, bs0.counter);
        //$monitor("rw0.state(%s) dataIn_t(%x) rw0.dataIn_t(%x) rw0.dataOut_p(%x)", rw0.state, dataIn_t, rw0.dataIn_t, rw0.dataOut_p);
        //$monitor("rw0.state(%s) rw0.done_p(%b) p0.state(%s) p0.readyToReceive_rw(%b) p0.done_enc(%b) enc0.inputReg(%b) enc0.pid(%s) nrzi0.outputBusState(%s) nrzi.outputValid(%b)", rw0.state, rw0.done_p, p0.state, p0.readyToReceive_rw, p0.done_enc, enc0.inputReg, enc0.pid, nrzi0.outputBusState, nrzi0.outputValid);
        //temprst <= 1;
              //$monitor("wires %b %b, nrzi out = %s, nrzioutputValid=%b, dec=%b, enc=%b \n dec packet = %b", wires.DP, wires.DM, nrzi_outputBusState, nrzi_outputValid, p0.done_dec, p0.done_enc, dnc_packet);
         //$monitor("dnc0.state(%s) p0.state(%s), encoder = %s, done = %b, dataOut_t=%h, datain=%h", dnc0.state, p0.state, enc0.state, rw0.done_p, p0.packetIn_dec, p0.lastPacketIn);
        //@(posedge clk); 
        memAddrIn_t <= mempage;
        txType_t <= 0;
        start_t <= 1;
        dataIn_t <= data;
        //@(posedge clk);
        wait(done_t);
        //@(posedge clk);
        start_t <= 0;
        success <= !error;
        @(posedge clk);
    endtask: writeData

    encoder enc0(clk, rst_L, enc_dataReady, enc_okToSend, packet, 
		 enc_busState, bs_dataReady, bs_stuffEnable, enc_readyToReceive);

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

    readwrite rw0(clk, rst_L, start_t, txType_t, memAddrIn_t, dataIn_t,
                 dataOut_t, done_t, txType_p, start_p, endpoint_p, dataIn_p,
                 dataOut_p, done_p);
    
    protocol p0(clk, rst_L, txType_p, start_p, enc_readyToReceive, dnc_dataReady,
                    done_p, error, enc_dataReady, endpoint_p, dataOut_p,
                    dataIn_p, packet, dnc_packet);

endmodule: usbHost

module readwrite(input logic clk, rst_L, start_t, txType_t,
                 input logic [15:0] memAddrIn_t,
                 input logic [63:0] dataIn_t,
                 output logic [63:0] dataOut_t,
                 output logic done_t,
                 output logic txType_p, start_p,
                 output logic [3:0] endpoint_p,
                 input logic [63:0] dataIn_p,
                 output logic [63:0] dataOut_p,
                 input logic done_p);
    
    //_t : talks to task
    //_p : talks to protocol
    //txtype 0 means output

    enum {WAIT, OUT_ADDR_READ, OUT_ADDR_READ_DONE, IN_DATA_READ, IN_DATA_READ_DONE, 
	  OUT_ADDR_WRITE, OUT_ADDR_WRITE_DONE, OUT_DATA_WRITE, OUT_DATA_WRITE_DONE, 
	  DONE} state, nextState;

    logic [15:0] memAddrIn_t_reg;
    logic [63:0] dataIn_t_reg;

    always_comb begin
        nextState = state;
        done_t = 0;
        start_p = 0;
        case(state)
	    //We first need to deside if we are reading or writing
            WAIT: begin 
                if(start_t) begin
                    if(txType_t) nextState = OUT_ADDR_READ;
                    else nextState = OUT_ADDR_WRITE;
                end
            end
            OUT_ADDR_READ: begin 
                nextState = OUT_ADDR_READ_DONE;
                txType_p = 0;
                dataOut_p = memAddrIn_t_reg << 48;
                endpoint_p = 4;
                start_p = 1;
            end
	    //The done states are used to wait for the protocol to finish
	    //sending and recieving data
            OUT_ADDR_READ_DONE: begin
                if(done_p) nextState = IN_DATA_READ;
            end
            IN_DATA_READ: begin 
                nextState = IN_DATA_READ_DONE;
                txType_p = 1;
                endpoint_p = 8;
                start_p = 1;
            end
            IN_DATA_READ_DONE: begin 
                if(done_p) nextState = DONE;
            end
            OUT_ADDR_WRITE: begin 
                nextState = OUT_ADDR_WRITE_DONE;
                txType_p = 0;
                dataOut_p = memAddrIn_t_reg<<48;
                endpoint_p = 4;
                start_p = 1;
            end
            OUT_ADDR_WRITE_DONE: begin
                if(done_p) nextState = OUT_DATA_WRITE;
            end
            OUT_DATA_WRITE: begin 
                nextState = OUT_DATA_WRITE_DONE;
                txType_p = 0;
                dataOut_p = dataIn_t;
                endpoint_p = 8;
                start_p = 1;
            end
            OUT_DATA_WRITE_DONE: begin
                if(done_p) nextState = DONE;
            end
            DONE: begin
                done_t = 1;
                nextState = WAIT;
            end
        endcase
    end

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            state <= WAIT;
            memAddrIn_t_reg <= 0;
            dataIn_t_reg <= 0;
        end
        else begin
            case(state)
                WAIT: begin
                    memAddrIn_t_reg <= memAddrIn_t;
                    dataIn_t_reg <= dataIn_t;
                end
                IN_DATA_READ_DONE: begin
                    if(done_p) dataOut_t <= dataIn_p;
                end
            endcase
            state <= nextState;
        end
    end


endmodule: readwrite

    
module protocol(input logic clk, rst_L, txType_rw, start_rw, done_enc, done_dec,
                output logic readyToReceive_rw, error, start_enc,
                input logic [3:0] endpoint_p,
                input logic [63:0] dataIn_rw,
                output logic [63:0] dataOut_rw,
                output logic [87:0] packetOut_enc,
                input logic [87:0] packetIn_dec);

    enum {WAIT, TOKEN_IN, TOKEN_IN_DONE, DATA_IN, DATA_IN_DONE, CRC_IN, NAK_IN, 
	  NAK_IN_DONE, ACK_IN, ACK_IN_DONE, TOKEN_OUT, TOKEN_OUT_DONE, DATA_OUT,
	  DATA_OUT_DONE, NAK_OUT, NAK_OUT_DONE, ACK_OUT, ACK_OUT_DONE} state, nextState;

    typedef enum logic[3:0] {OUT = 4'b0001, IN = 4'b1001, DATA0 = 4'b0011,
                             ACK = 4'b0010, NAK = 4'b1010} pidValue;

    logic [63:0] dataIn_rw_reg;
    logic [7:0] crcCounter, timeoutCounter, errorCounter;
    logic startCRC5, startCRC16, crcValid;
    logic [15:0] crc, crc16Result;
    logic [4:0] crc5Result;
    logic [87:0] lastPacketIn;
    pidValue pid;
    //The crcs below are used to calculate the residuals
    crc5 #(1) a1(clk, rst_L, lastPacketIn[23:8], startCRC5, crc5Result);
    crc16 #(1) a2(clk, rst_L, lastPacketIn[87:8], startCRC16, crc16Result);

    always_comb begin
        pid = pidValue'(packetIn_dec[3:0]);
        crc = (pid == DATA0) ? crc16Result : crc5Result;
        nextState = state;
        //We need to ensure that we finish all tasks before we start a new one
        readyToReceive_rw = state == WAIT && done_enc && done_dec;
        start_enc = 0;
        //This is used to check the residual value
        crcValid = (pid == DATA0) ? (crc == 16'h800D) : (crc == 5'b01100);
        dataOut_rw = lastPacketIn[71:8];
        startCRC5 = 0;
        startCRC16 = 0;
        case(state)
            WAIT: begin
                if(start_rw) nextState = txType_rw ? TOKEN_IN : TOKEN_OUT;
            end
            TOKEN_IN: begin
                start_enc = 1;
                packetOut_enc = {endpoint_p, 7'd5, 4'b0110, 4'b1001};
                if(!done_enc) nextState = TOKEN_IN_DONE;
            end
	    //This state and all of the done states are used to have the fsm wait until
	    //the encoder or decoder are done
            TOKEN_IN_DONE: begin
                if(done_enc) nextState = DATA_IN;
            end
	    //We need to check for errors and stop if we see 8 errors
            DATA_IN: begin
                if(errorCounter == 8) nextState = WAIT;
                else if(!done_dec) nextState = DATA_IN_DONE;
            end
            DATA_IN_DONE: begin
	        //We also want to ensure that if we dont hang so we have a timeout
                if(done_dec) nextState = CRC_IN;
                else if(timeoutCounter == 255) nextState = NAK_IN;
            end
            CRC_IN: begin
                startCRC5 = (crcCounter == 0) && (pid != DATA0);
                startCRC16 = (crcCounter == 0) && (pid == DATA0);
                if((pid == DATA0 && crcCounter == 81) ||
                   (pid != DATA0 && crcCounter == 17)) 
                    nextState = (crcValid) ? ACK_IN : NAK_IN;
            end
            NAK_IN: begin
                start_enc = 1;
                packetOut_enc = {4'b0101, 4'b1010};
                if(!done_enc) nextState = NAK_IN_DONE;
            end
            NAK_IN_DONE: begin
                if(done_enc) nextState = DATA_IN;
            end
            ACK_IN: begin
                start_enc = 1;
                packetOut_enc = {4'b1101, 4'b0010};
	        if(!done_enc) nextState = ACK_IN_DONE;
            end
            ACK_IN_DONE: begin
                if(done_enc) nextState = WAIT;
            end
            TOKEN_OUT: begin
                start_enc = 1;
                packetOut_enc = {endpoint_p, 7'd5, 4'b1110, 4'b0001};
                if(!done_enc) nextState = TOKEN_OUT_DONE;
            end
            TOKEN_OUT_DONE: begin
                if(done_enc) nextState = DATA_OUT;
            end
            DATA_OUT: begin
                start_enc = 1;
                packetOut_enc = {dataIn_rw_reg, 4'b1100, 4'b0011};
                if(errorCounter == 8) nextState = WAIT;
                else if(!done_enc) nextState = DATA_OUT_DONE;
            end
            DATA_OUT_DONE: begin
                if(done_enc) nextState = ACK_OUT;
            end
            ACK_OUT: begin
                if(!done_dec) nextState = ACK_OUT_DONE;
            end
            ACK_OUT_DONE: begin
                if(done_dec) begin
                    if(pid == ACK) nextState = WAIT;
                    else nextState = DATA_OUT;
                end
            end
        endcase
    end

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            errorCounter <= 0;
            timeoutCounter <= 0;
            crcCounter <= 0;
            lastPacketIn <= 0;
            error <= 0;
        end

        else begin
            if(nextState == WAIT && errorCounter == 8) begin
                error <= 1;
            end
            case(state)
                WAIT: begin
                    errorCounter <= 0;
                    if(nextState != WAIT) begin
                        dataIn_rw_reg <= dataIn_rw;
                        error <= 0;
                    end
                end
                TOKEN_IN: begin
                    timeoutCounter <= 0;
                end
                DATA_IN: begin
                    timeoutCounter <= timeoutCounter + 1;
                    crcCounter <= 0;
                end
                DATA_IN_DONE: begin
                    timeoutCounter <= timeoutCounter + 1;
                    if(done_dec) begin
                        lastPacketIn <= packetIn_dec;
                    end
                end
                CRC_IN: begin
                    crcCounter <= crcCounter + 1;
                end
                NAK_IN: begin
                    timeoutCounter <= 0;
                    errorCounter <= errorCounter + 1;
                end
                ACK_OUT: begin
                    if(done_dec && pid != ACK) errorCounter <= errorCounter + 1;
                end
            endcase
            state <= nextState;
        end

    end

    // txType: 1 => input, 0 => output


endmodule: protocol

/*
 The encoder module is used to take a packet, look at its PID, and based on
 that PID it will send either the first 24 bits or all 88 bits. It will also
 first send the sync bits and will end with the EOP bits.
 */
module encoder
    (input logic clk, rst_L, dataReady, okToSend,
     input logic [87:0] packet,
     output busState outputBusState,
     output logic outputReady, stuffEnable, readyToReceive);

    logic [7:0] index; //Index counter
    logic [87:0] inputReg; //Holds our data
    logic [15:0] crc, crc16Result;
    logic [7:0] counter;
    logic [4:0] crc5Result;
    logic crc5Enable, crc16Enable;
	   
    enum {WAIT, SYNC, DATA, CRC, EOP} state, nextState;
    typedef enum logic[3:0] {OUT = 4'b0001, IN = 4'b1001, DATA0 = 4'b0011,
                     ACK = 4'b0010, NAK = 4'b1010} pidValue;
    pidValue pid;

    crc5 #(0) a1(clk, rst_L, packet[18:8], crc5Enable, crc5Result);
    crc16 #(0) a2(clk, rst_L, packet[71:8], crc16Enable, crc16Result);

    always_comb begin
        crc = (pid == DATA0) ? ~crc16Result : ~crc5Result;
        outputBusState = bus_J;
        outputReady = (state != WAIT);
        pid = pidValue'(inputReg[3:0]);
        stuffEnable = 0;
        readyToReceive = 0;
        crc5Enable = 0;
        crc16Enable = 0;
        case(state)
            // WAIT: Stall until we receive a packet
            WAIT: begin
                nextState = (dataReady) ? SYNC : WAIT;
                readyToReceive = 1;
            end
            // SYNC: Send the sync byte (00000001)
            SYNC: begin
                if(counter == 8'd7) begin
                    nextState = DATA;
                    outputBusState = bus_J;

                    if(pid == DATA0) crc16Enable = 1;
                    else crc5Enable = 1;
                end
                else begin
                    nextState = SYNC;
                    outputBusState = bus_K;
                end
            end
            // DATA: Increment a counter to transmit the packet payload one bit at a time
            DATA: begin
                outputBusState = busState'(inputReg[index]);
                stuffEnable = index >= 7;
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
            if(okToSend) begin
                if(state == WAIT) begin
                    counter <= 0;
                    index <= 0;
                end
                if(state == WAIT && nextState == SYNC) begin
                    inputReg <= packet;
                end
                if(state == SYNC) begin
                    if(nextState == DATA) begin
                        counter <= 0;
                    end
		    else counter <= counter + 1;
                end
                if(state == DATA) begin
                    index <= index + 1;
                    if(nextState == CRC) begin
                        if(pid == DATA0) counter <= 15;
                        else counter <= 4;
                    end
                end
                if(state == CRC) begin
                    counter <= counter - 1;
                end
                if(state != EOP && nextState == EOP) begin
                    counter <= 0;
                end
                if(state == EOP) begin
                    counter <= counter + 1;
                end
                state <= nextState;
            end
        end
    end
   
endmodule : encoder

module crc5
    (input logic clk, rst_L,
     input logic [15:0] data,
     input logic dataReady,
     output logic [4:0] crc);

    parameter CALC_RESIDUE = 0;

    enum {WAIT, GO} state, nextState;

    logic [15:0] dataReg;
    logic [4:0] counter;

    logic crc0_next, crc2_next;

    always_comb begin
        case(state)
            WAIT: nextState = (dataReady) ? GO : WAIT;
            GO: begin
                if(CALC_RESIDUE)
                    nextState = (counter == 5'd15) ? WAIT : GO;
                else
                    nextState = (counter == 5'd10) ? WAIT : GO;
            end
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
     input logic [79:0]  data,
     input logic dataReady,
     output logic [15:0] crc);
    
    parameter CALC_RESIDUE = 0;

   
    enum {WAIT, GO} state, nextState;

    logic [79:0] dataReg;
    logic [7:0] counter;
    logic 	crc0_next, crc2_next, crc15_next;
   
    always_comb begin
        case(state)
            WAIT: nextState = (dataReady) ? GO : WAIT;
            GO: begin
                if(CALC_RESIDUE)
                    nextState = (counter == 8'd79) ? WAIT : GO;
                else
                    nextState = (counter == 8'd63) ? WAIT : GO;
            end
        endcase // case (state)
        crc0_next = dataReg[counter]^crc[15];
        crc2_next = crc0_next^crc[1];
        crc15_next = crc0_next^crc[14];
    end

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            state <= WAIT;
            crc <= 16'hFFFF;
            counter <= 0;
        end
        else begin
            if(state == WAIT && nextState == GO) begin
                dataReg <= data;
                crc <= 16'hFFFF;
                counter <= 0;
            end
            else if(state == GO) begin
                crc[0] <= crc0_next;
                crc[1] <= crc[0];
                crc[2] <= crc2_next;
                crc[14:3] <= crc[13:2];
                crc[15] <= crc15_next;
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
    logic stuffEnableLatch;

    always_comb begin
        outputReady = dataReady || (counter == 6);
        outputBusState = (counter == 6) ? bus_K : inputBusState;   
        readyToReceive = !(counter == 6);
    end

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            counter <= 0;
            stuffEnableLatch <= 0;
        end   
        else begin
            stuffEnableLatch <= stuffEnable;
            if(counter == 6) begin
                counter <= 0;
            end
            else if(dataReady) begin
                if(stuffEnableLatch && inputBusState == bus_J) counter <= counter + 1;
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
                else if(inputBusState == bus_K) outputBusState <= (outputBusState == bus_K) ? bus_J : bus_K;
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
            if(counter == 6 && dataReady && unstuffEnable) begin
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
                nextState = (counter >= 8'd7 && inputBusState == bus_J) ? DATA : SYNC;
	    //    unstuffEnable = (counter >= 8'd7);
            end
            // DATA: Increment counter to recieve the data into packet
            DATA: begin
                unstuffEnable = (index >=8);
	        nextState = DATA;
                case(pid)
                    OUT: begin
                        if(index >= 8'd23) nextState = EOP;
                    end
                    IN: begin
                        if(index >= 8'd23) nextState = EOP;
                    end
                    DATA0: begin
                        if(index >= 8'd87) nextState = EOP;
                    end
                    ACK: begin
                        if(index >= 8'd7) nextState = EOP;
                    end
                    NAK: begin
                        if(index >= 8'd7) nextState = EOP;
                    end
                endcase // case (pid)
            end
            // CRC: Takes in the CRC of the payload 
            CRC: begin
                unstuffEnable = 1;
	        nextState = (counter == 0) ? EOP : CRC;
            end
            // EOP: Send EOP signal
            EOP: begin
                if(counter >= 8'd2) begin
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
		    index <= 0;
		    outputReg <= 0;
		    outputReady <= 0;
		    //Currently the output is still the old packet 
		    //which is valid until the sync
		end
	        //Then we wait until we see a stream of at least 7 zeros
	        else if(inputBusState == bus_K) begin
		    counter <= counter + 1;
		end
	        //If we get a zero before this, we reset counter
	        else if(counter < 8'd7 && inputBusState != bus_K) begin
		    counter <= 0;
		end
            end
            if(state == DATA && dataReady) begin
	        //We need to assign data to the inputReg 
	        //by converting the state into a 1 or 0
                outputReg[index] <= logic'(inputBusState);
                index <= index + 1;
                /*
                if(inputBusState == bus_J) begin
                    outputReg[index] <= 1;
                end
                else begin
                    outputReg[index] <= 0;
                end
                */
            end
            if(nextState == EOP && state != EOP) begin
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
endmodule: decoder
