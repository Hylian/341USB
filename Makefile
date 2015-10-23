CC=vcs

FLAGS=-sverilog -debug

default: full

student: top.sv usbBusAnalyzer.svp tb.sv usbHost.sv thumb.svp
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp tb.sv usbHost.sv thumb.svp

simple: top.sv usbBusAnalyzer.svp TA_tb_simple.svp usbHost.sv thumb.svp
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp TA_tb_simple.svp usbHost.sv thumb.svp

full: top.sv usbBusAnalyzer.svp TA_tb_full.svp usbHost.sv thumb.svp
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp TA_tb_full.svp usbHost.sv thumb.svp

faulty: top.sv usbBusAnalyzer.svp TA_tb_faults.svp usbHost.sv thumb.svp
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp TA_tb_faults.svp usbHost.sv thumb.svp

prelab: top.sv usbBusAnalyzer.svp TA_tb_prelab.svp usbHost.sv thumb.svp
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp TA_tb_prelab.svp usbHost.sv thumb.svp

clean:
	rm -rf simv
	rm -rf simv.daidir
	rm -rf csrc
	rm -rf ucli.key
	rm -rf simv.vdb
	rm -rf DVEfiles
	rm -rf inter.vpd
