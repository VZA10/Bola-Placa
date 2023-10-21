# Bola-Placa
Implementação de um controle discreto por alocação de polos com um observador de ordem reduzida em um sistema Bola-Placa


Este trabalho aborda o projeto de controle de um sistema Bola-Placa didático. O sistema promove a integração de conhecimentos multidisciplinares por ser eletromecânico, multivariável,
não linear e naturalmente instável. O trabalho apresenta o levantamento de materiais, parâmetros
e o projeto de um controlador discreto por alocação de polos. Dois sensores são usados, uma
câmera para capturar a posição da bola sobre a placa e um acelerômetro combinado com um
giroscóprio para medir o ângulo atual da placa, além disso, é utilizado um observador de ordem
reduzida para estimar outros dois estados do sistema. Com o controlador aplicado na planta
real é possível impor à bola respostas pontuais e o rastreamento de trajetórias, os resultados
práticos do posicionamento da bola são similares ao simulado. No entanto por conta de incertezas no modelo, não linearidades e problemas apresentados pelo sensor do ângulo da placa, há
divergências nas variáveis observadas, que acabam por interferir na resposta.

Palavras-chave: Controle Moderno; Observador de Estados; Protótipo de Sistema Bola-Placa
Didático.
