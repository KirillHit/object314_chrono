FROM kirillhit/project_chrono:0.1

ENV LD_LIBRARY_PATH=/opt/vsg/lib:$LD_LIBRARY_PATH
RUN apt install -y vulkan-tools

RUN apt update && apt install -y build-essential cmake gdb clang python3 libclang-dev