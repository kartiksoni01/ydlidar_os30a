#!/bin/sh
echo "run x86_64"
export LD_LIBRARY_PATH=../eSPDI:$LD_LIBRARY_PATH 
export LD_LIBRARY_PATH=../eSPDI/opencv/x86_64/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=../eSPDI/turbojpeg/x86_64/lib/:$LD_LIBRARY_PATH 
sync

strace_log_file=eysdbg_strace.txt
kernel_log_file=eysdbg_kernel.txt
gdb_log_file=eysdbg_gdb_bt.txt

strace_dm_preview() {
        strace -t -e trace=signal -p $(pgrep DMPreview_X86) 2>&1 | tee $strace_log_file
}

watch_termination() {
    while true
    do
        dm_preview_terminate_reason=$(cat $strace_log_file | tail -n 1)
        case "$dm_preview_terminate_reason" in
            *exited*)
                return 0
                ;;
            *killed*)
                dmesg > $kernel_log_file
                gdb DMPreview_X86 core -ex 'bt' -ex 'q' > $gdb_log_file
                return 0
                ;;
            *)
                sleep 1
                ;;
        esac
    done
}

ulimit -c unlimited
./DMPreview_X86 &
strace_dm_preview
watch_termination
