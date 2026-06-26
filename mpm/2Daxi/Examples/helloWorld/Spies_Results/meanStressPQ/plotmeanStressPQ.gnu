set bmargin 6
set lmargin 12
set rmargin 12
#set term context {font "Helvetica,20"}
plot 'meanstressPQ.txt' u 6:7 with points lt -1 pt 7 pointsize 1.5 lc rgb 'red' title 'meanStress',\
'meanstressPQ.txt' u 6:7 with lines lc rgb 'red' ,\
'meanstressPQ.txt' u 6:8 w l title 'DP yield surface'
set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20"
set title font "Helvetica,20"
set xrange [* : *] noreverse nowriteback
set xlabel 'P'
set ylabel 'Q'
