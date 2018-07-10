#!/bin/bash

#set -x
if [  $# -ne 3 ];then
    echo "please input three num"
    echo "第一个参数为talker：$1";
    echo "第二个参数为listener：$2";
    echo "第三个参数为bw：$3";
    exit 1;
fi
echo "执行的脚本名：$0";
echo "第一个参数为talker：$1";
echo "第二个参数为listener：$2";
echo "第三个参数为bw：$3";
tknum=$1;
ltnum=$2;
bwnum=$3;
echo "<launch>" > "talker${1}_listener${2}_bw${3}.launch"
for ((i=0;i<$1;i++));
do
    if [ $bwnum -ge 1024000 ];then
         rate=2
         time=300s
         num=100
    else
         rate=200
         time=60s
         num=1000
    fi
    echo " <node name=\"talker_$i\" pkg=\"rosperf\" type=\"perf_talker\"  args=\"$bwnum $num $rate 1\" output=\"screen\" />"  >> "talker${1}_listener${2}_bw${3}.launch";
done

for ((i=0;i<$2;i++));
do
    echo " <node name=\"listener_$i\" pkg=\"rosperf\" type=\"perf_listener\"  args=\"0\" output=\"screen\" />" >> "talker${1}_listener${2}_bw${3}.launch";
done

echo '</launch>' >> "talker${1}_listener${2}_bw${3}.launch";
timeout $time roslaunch rosperf talker${1}_listener${2}_bw${3}.launch 