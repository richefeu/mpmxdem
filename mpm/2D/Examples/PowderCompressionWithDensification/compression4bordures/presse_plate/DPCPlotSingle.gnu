set bmargin 6
set lmargin 12
set rmargin 12


plot 'DPCCurve.txt' u 1:2 w l linewidth 3,\
'DPCPlotSingleMP350.txt' u 2:3 with points pt 7 pointsize 1.5 lc rgb 'red',\
'DPCPlotSingleMP350.txt' u 2:3 with line linewidth 3 lc rgb 'red',\
'DPCPlotSingleMP350.txt' u ($2+30):($3+30):0 with labels font 'Helvetica,8' ,\


#set grid back lc rgb 'black' linewidth 1.5
set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20" offset 0,4
set xtics font "Helvetica,20"
set ytics font "Helvetica,20"
set ytics 1000

set title font "Helvetica,30"
set xrange [0 : 3345] 
set yrange [0 : 3618]
set xlabel 'P'
set ylabel 'Q'
