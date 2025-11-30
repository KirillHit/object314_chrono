FROM kirillhit/project_chrono:0.2

RUN apt update && apt install -y build-essential cmake

WORKDIR /workspace
COPY ../src .

RUN mkdir -p build \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --target install --parallel