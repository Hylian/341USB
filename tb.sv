
module tb
  (output logic clk, rst_L);
   bit [15:0] addr;
   bit [63:0] dataRead;
   bit [63:0] dataWrite;
   bit 	      did_it_right;
   bit [7:0]  errorCount;


   
   //first we start the clk
   always begin
      forever clk = #1 ~clk;
   end
   
   initial begin
      rst_L = 1;
      @(posedge clk);
      rst_L = 0;
      @(posedge clk);
      rst_L = 1;
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
      #5 writeData(addr, dataWrite, did_it_right);
      @(posedge clk);
      if(did_it_right) begin
	 $display("Test 2: Passed");
      end
      else begin
	 errorCount <= errorCount + 1;
	 $display("Test 2: Failed - no success");
	 $display("addr = %h \n data=%h", addr, dataWrite);
      end
   end // initial begin
   
endmodule : tb
