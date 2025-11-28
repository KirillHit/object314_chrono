FROM kirillhit/project_chrono:0.2

RUN apt update && apt install -y build-essential cmake gdb libclang-dev clang-format sudo


ARG UID=1000
ARG GID=1000
ARG USERNAME=user

RUN groupadd -g $GID $USERNAME || true \
    && useradd -m -u $UID -g $GID -s /bin/bash $USERNAME || true \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash \
    && sudo apt update \
    && sudo apt install -y git-lfs

CMD ["/bin/bash"]