# RMA_LineFollower

Este repositório contém os arquivos do CoppeliaSim e o código necessário para rodar um seguidor de linha com possibilidade de retorno ao começo do percurso, caso ele tenha se perdido, ou tenha finalizado-o. 

Este trabalho foi realizado para a disciplina SSC0714 - Robôs Móveis Autônomos (2022) para o curso de Engenharia de Computação na Universidade de São Paulo - Campus São Carlos.

# Descrição do repositório

Este repositório consiste em uma pasta cppCode com o código em C++ do robô e um código em C para utilização do remoteAPI do CoppeliaSim, possibilitando comunicação do simulador com o código em C++, obtido de http://osorio.wait4.org/VREP/Programacao/CPPremoteAPI/.

# Dependências

Este código foi programado em **Ubuntu 20.04**, utilizando o ambiente de simulação **CoppeliaSim V4.3.0**. 

Além disso, utilizou-se funções do **OpenCV**. Para instalação, foi seguido o tutorial no seguinte link: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html

Caso queira apenas os comandos do terminal, siga os passos abaixo:

```
sudo apt update && sudo apt install -y cmake g++ wget unzip

wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip

mkdir -p build && cd build

cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x

cmake --build .

sudo make install
```

# Compilação
Para compilação do programa

```
cd <path_to_rma_line_follower>/cppCode/

cmake .

make
```

# Execução

Para execução da simulação, o CoppeliaSim deve estar aberto com o arquivo *scene.ttt*.

```
cd <path_to_rma_line_follower>/cppCode/
./cppremoteapi
```
