10 REM Retro Desk Mandelbrot Benchmark
50 CLS
90 T=TICKS()
100 FOR A=0 TO 21
110 FOR B=0 TO 31
120 C=(B*0.10938)-2.5
130 D=(A*0.09090)-1.0
140 X=0
150 Y=0
160 FOR I=0 TO 14
165 F=X*X+Y*Y
170 IF F>4 EXIT
180 E=X*X-Y*Y+C
190 Y=2*X*Y+D
200 X=E
210 NEXT I
220 PRINT CHR$(I+126);
240 NEXT B
255 print ""
260 NEXT A
265 PRINT "Elapsed time: ";
270 PRINT (TICKS()-T)/60;
280 PRINT " Seconds"