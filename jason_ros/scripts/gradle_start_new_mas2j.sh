#!/bin/bash
echo "MAS $2 {

    infrastructure: Centralised

    agents: $2 $3 agentArchClass jasonros.RosArch;
}" > $1/$2.masj

gradle run -b $1/build.gradle -Pmas2j=$1/$2.masj
