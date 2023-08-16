MODEL=testdata/model/LowerJawScan_1M.ply
MODEL=testdata/model/LowerJawScan_100k.ply

tand-1minus:
	crop3d -xh 10 -xl -5 -yl -17 -yh 0 -zl 15 -zh 31 -o tmp/tand-1minus.ply $(MODEL) 


scan-1minus:
	
clean:
	rm -rf tmp/*
