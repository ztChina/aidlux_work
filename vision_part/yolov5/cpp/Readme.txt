Use the steps Description：

1.  sudo apt update
    sudo apt-get install cmake -y

# Our sample program in [/usr/local/share/aidlite/examples/] directory.
# We recommend that you copy the sample program directory to your own working directory.
2.  mkdir -p /home/aidlux/your_work_dir/ && cp -r /usr/local/share/aidlite/examples/ /home/aidlux/your_work_dir/
    cd /home/aidlux/your_work_dir/examples/aidlite_qnn/cpp/

# If you do not want to copy the sample program, then next few steps require Sudo permissions
3.  mkdir -p build && cd build
4.  cmake ..
5.  make 

# Run the resulting executable program
6.  /xxx.exe