Please download the kortex_api zip folder from https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/API/2.5.0/kortex_api_2.5.0.zip

Then unzip that file,

then run something like the below to copy the needed files into a new directory called kinova_gen3_control/kortex_api/linux_gcc_x86-64

cd ~/Downloads
rm -rf kortex_api
wget https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip
unzip linux_x86-64_x86_gcc.zip -d kortex_api
cp -r ~/Downloads/kortex_api/* ~/custom_trajectory_ws/src/kinova_gen3_control/kortex_api/
