#!/bin/bash
echo "Start rosperf test"
pub=(1 4 8)
sub=(1 8 20)
bw=(64 128 1024 1024000 5120000 10240000)

for (( i=0; i<${#pub[@]}; ++i ));
do
echo "latency rosperf">>rosperf.csv
        for (( j=0; j<${#sub[@]}; ++j ));
        do
           echo "pub ${pub[i]} sub ${sub[j]}" >>rosperf.csv
           echo "bw,max,min,avg,stdev">>rosperf.csv
                for (( z=0; z<${#bw[@]}; ++z ));
                do
                echo "pub="${pub[i]} "sub="${sub[j]} "bw=" ${bw[z]};
                file=local_perf_${pub[i]}_${sub[j]}_${bw[z]}.log
                ./create_launch.sh ${pub[i]} ${sub[j]} ${bw[z]} | tee $file
                sleep 5
                echo "##############################################"
                echo "pub="${pub[i]} "sub="${sub[j]} "bw=" ${bw[z]} "test done."
                sleep 5
                echo "##############################################"
                #echo "#######calc data and save to file#############">> output/rosperf.log
                #file=local_perf_${pub[i]}_${sub[j]}_${bw[z]}.log
                #cacl_data($file)
                #echo "print data"
                grep -nr "^/talker_[0-9]-" $file | awk '{print $3}'>max.txt
                max1=`sed '/^$/d' max.txt|awk 'NR==1{max=$1;next}{max=max>$1?max:$1}END{print max}'`
                grep -nr "^/talker_[0-9]-" $file | awk '{print $4}' >min.txt
                min1=`sed '/^$/d' min.txt|awk 'NR==1{min=$1;next}{min=min<$1?min:$1}END{print min}'`
                grep -nr "^/talker_[0-9]-" $file | awk '{print $5}'>avg.txt
                avg1=`cat avg.txt |awk '{a+=$1}END{print a/NR}'`
                grep -nr "^/talker_[0-9]-" $file | awk '{print $6}'>std.txt
                stdev1=`cat std.txt |awk '{a+=$1}END{print a/NR}'`
                #echo "pub${pub[i]}_sub${sub[j]}_bw${bw[z]} rosperf data is:" >> output/rosperf.log
                #echo "max:$max1, min:$min1, avg:$avg1, stdev:$stdev1" >> output/rosperf.log
               echo ${bw[z]},$max1,$min1,$avg1,$stdev1 >>rosperf.csv
               done
           echo "" >>rosperf.csv
        done
done