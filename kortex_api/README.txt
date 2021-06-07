Please download the kortex_api zip folder from https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/API/2.0.0/kortex_api_2.0.0.zip

Then unzip that file,

then run something like the below to copy the needed files into a new directory called kinova_gen3_control/kortex_api/linux_gcc_x86-64

cd ~/Downloads
wget https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/API/2.0.0/kortex_api_2.0.0.zip
unzip kortex_api_2.0.0.zip -d kortex_api
cp -r ~/Downloads/kortex_api/cpp/linux_gcc_x86-64/ ~/untitled_ws/src/kinova_gen3_control/kortex_api/
cd ~/untitled_ws
