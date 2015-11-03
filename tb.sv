
module test
  (output logic clk, rst_L);
   bit [15:0] addr;
   bit [63:0] dataRead;
   bit [63:0] dataWrite;
   bit 	      did_it_right;
   bit [7:0]  errorCount;


   
   //first we start the clk
   always begin
      clk = 1;
      
      forever clk = #1 ~clk;
   end
   
   initial begin
      rst_L <= 1;
      @(posedge clk);
      rst_L <= 0;
      @(posedge clk);
      rst_L <= 1;
      @(posedge clk);
      $display("****************************************");
      $display("****************Write Test***************");
      $display("****************************************");
      
      errorCount <= 0;
      addr <= 16'hFFFF;
      dataWrite <= 64'hFFFF_FFFF_FFFF_FFFF;
      @(posedge clk);
      host.writeData(addr, dataWrite, did_it_right);
      @(posedge clk);
      if(did_it_right) begin
	 $display("Test 1: Passed");
      end
      else begin
	 errorCount <= errorCount + 1;
	 $display("Test 1: Failed - no success");
	 $display("addr = %h \n data=%h", addr, dataWrite);
      end
      @(posedge clk);
      addr <= 16'h0001;
      dataWrite <= 64'h0000_0000_0000_0100;
      @(posedge clk);
      host.writeData(addr, dataWrite, did_it_right);
      @(posedge clk);
      if(did_it_right) begin
	 $display("Test 2: Passed");
      end
      else begin
	 errorCount <= errorCount + 1;
	 $display("Test 2: Failed - no success");
	 $display("addr = %h \n data=%h", addr, dataWrite);
      end
      $display("****************************************");
      $display("****************Read Test***************");
      $display("****************************************");
      
      addr <= 16'hFFFF;
      @(posedge clk);
      host.readData(addr, dataRead, did_it_right);
      @(posedge clk);
      if(did_it_right && dataRead == 64'hFFFF_FFFF_FFFF_FFFF) begin
	 $display("Test 3: Passed");
      end
      else begin
	 errorCount <= errorCount + 1;
	 $display("Test 3: Failed - no success");
	 $display("addr = %h \n data=%h", addr, dataRead);
      end
      
      @(posedge clk);
      #100 $finish;
      $display("****************************************");
      $display("*************Bad Mem Test***************");
      $display("****************************************");
      
      addr <= 16'h1234;
      @(posedge clk);
      host.readData(addr, dataRead, did_it_right);
      @(posedge clk);
      if(did_it_right && dataRead == 0) begin
	 $display("Test 4: Passed");
      end
      else begin
	 errorCount <= errorCount + 1;
	 $display("Test 4: Failed - no success");
	 $display("addr = %h \n data=%h", addr, dataRead);
      end
      
      @(posedge clk);
   end // initial begin
   
endmodule : test
