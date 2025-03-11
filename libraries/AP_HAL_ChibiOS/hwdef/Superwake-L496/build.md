# folder
/Users/ayeung/Documents/Git/superwake-ardupilot/build/Superwake-L496

# branch
superwake-ap-periph


python ./waf configure --board Superwake-L496 --bootloader
python Tools/scripts/build_bootloaders.py Superwake-L496

python ./waf configure --board Superwake-L496
python ./waf AP_Periph

# build sitl
python ./waf configure --board sitl
python ./waf plane

# run sitl
python Tools/autotest/sim_vehicle.py -v ArduPlane -S 1 --out 127.0.0.1:10034 --custom-location 60.855009,-135.519245,0,0 --add-param-file Tools/autotest/models/sw117.parm


# mac install
pip install empy==3.3.4
pip install intelhex
brew install --cask gcc-arm-embedded
pip install pexpect==4.9.0


from intelhex import bin2hex
ModuleNotFoundError: No module named 'intelhex'
Build failed: /Users/ayeung/.pyenv/versions/3.9.14


## launch dronecan gui tool
dronecan_gui_tool
