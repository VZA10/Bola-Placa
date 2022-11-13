import serial
import numpy as np
import time
import cv2
import keyboard
import math
import threading



testedeangulos = False
circulo =False
quadrado =True


#setpoint em centimetros aproximados m*21.2
setpoint_x=0 #centimetros
setpoint_y=0

centro_da_bola=[setpoint_x,setpoint_y]

setpoint_xp=0 #pixel
setpoint_yp=0

#distandia entre braço articulador e eixo
eixo_motor=2.5
eixo_mesa=8
ajuste_angulos=1 #-->multiplica conversão
zero=False

#faixa de cor da bola gude
bolaLower = (49, 35, 125)   
bolaUpper = (95,90,180)
#faixa de cor da bola ping pong
'''bolaLower = (0, 85, 150)   
bolaUpper = (20,180,255)'''

fps=0.06 #tempo do controle
#variaveis motor
motor_x=0
motor_y=0
motor=0
motor1=85
motor2=87
motorx=87
motory=85
mesa1=0
mesa2=0


#dados do video
cap=cv2.VideoCapture(1)#, cv2.CAP_DSHOW
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

conv= 30 #area que a camera pega x,y / depende da altura da camera
angulomax=math.radians(angulomax)
multiplicador_mesa=1
center=[0,0]
frame= None

#variaveis controle do motor
kc=1
b0=2.295
b01=-4.373
b02=2.079
a01=-1.482
a02=0.4849
x00=0
x01=0
x02=0

#constantes realimentação de estados
#ki=0.04 #para integrativo

k0=1
k1=0.66
k2=0.414542
k3=-0.108090
k4=-0.037114

controle0=0

valores=[] #varialvel da serial


#salava dados
vetor_posicao1=[]
vetor_posicao2=[]
vetor_angulo_x=[]
vetor_angulo_y=[]
vetor_tempo=[]
vetor_setpoint_x=[]
vetor_setpoint_y=[]
vetor_motor_x=[]
vetor_motor_y=[]

vetor_velocidade_Ax=[]
vetor_velocidade_Ay=[]
vetor_velocidade_Bx=[]
vetor_velocidade_By=[]

#variacao_inicial=True #para calculo das primeiras acelerações 

valores1=['null','null','null','null']
valores=[0,0,0,0]

'''
#constantes MPU6050
#para converter em força g as leituras do acelerômetro
Acc = 16384
#para converter em DPS (graus por segundo) leituras do giroscópio
Gy = 131

Ax=0
Ay=0
Az=0

#velocidades iniciais
velocidadeAx0=0
velocidadeAy0=0
velocidadeBx0=0
velocidadeBy0=0

#posição inicial
centrodabola00=0
centrodabola01=0
'''

'''#matrizes observador de ordem reduzida ##3variaveis observadas
Amm=1
Aom=np.array([[0],[0],[0]])
Amo=np.array([ 0.0399,-0.0054,-0.0001])
Aoo=np.array([[0.9968,-0.2656,-0.0038],
             [0,   0.8617,   0.0219],
             [0,  -5.5852,   0.2117]])
Bo=np.array([[ -1.4257*10**-2],[1.3825*10**-1],[5.5852]])
Bm= np.array([-1.5118*10**-4])
G=np.array([[5.7571e-03],[-2.1690e-05] ,[2.1908e-04]])
xh=np.array([[0],[0],[0]])
posicao0=0
controle=0

ukx=0
uky=0'''

#matrizes observador de ordem reduzida ##2variaveis observadas
Amm=np.array([[1,-0.01278],[0,0.7488]])
Amo=np.array([[0.06192,-0.0002033],[0,0.03151]])
Aom=np.array([[0,-0.3932],[0,-6.308]])
Aoo=np.array([[0.9757,-0.008785],[0,0.12911]])

Bo=np.array([[-0.0407],[6.308]])
Bm= np.array([[-0.0006734],[0.2512]])

G=np.array([[15.516,-0.17688],[-0.37048,3.5474]])

xh2=np.array([[0],[0]])
xh1=np.array([[0],[0]])

posicao0=0
posicao01=0
controle0=np.array([[0],[0]])
controle2=0
angulo0=0
controle1=0
angulo01=0
ukx=0
uky=0
vbx=0
vax=0
vby=0
vay=0


#Vetor Circulo
velocidade=4
raio=6
pointsListCirclex = []
pointsListCircley = []
ix=0
for angle_ in range(0, 360):
        angle_ = angle_ - 90
        pointsListCirclex.append(raio * math.cos(math.radians(angle_)) + 0)
        pointsListCircley.append(raio * math.sin(math.radians(angle_)) + 0)

#Vetor Quadrado
tamanho=15
velocidade=6
pointsListQuadradox = []
pointsListQuadradoy = []
ix1=0
for qx1 in range (0,tamanho*7):
    pointsListQuadradox.append((0-tamanho/2)+(qx1/7))
    pointsListQuadradoy.append(0-tamanho/2)
for q1 in range (0, tamanho*7):
    pointsListQuadradox.append((0+tamanho/2))
    pointsListQuadradoy.append(0-tamanho/2)


for qy1 in range (0,tamanho*7):
        pointsListQuadradoy.append((0-tamanho/2)+(qy1/7))
        pointsListQuadradox.append(0+tamanho/2)
for qx0 in range (0, tamanho*7):
    pointsListQuadradox.append((0+tamanho/2))
    pointsListQuadradoy.append(0+tamanho/2)

    
for qx2 in range (0,tamanho*7):
        pointsListQuadradox.append(0+tamanho/2-(qx2/7))
        pointsListQuadradoy.append(0+tamanho/2)
for q3 in range (0, tamanho*7):
    pointsListQuadradox.append((0-tamanho/2))
    pointsListQuadradoy.append(0+tamanho/2)


for qy2 in range (0,tamanho*7):
        pointsListQuadradoy.append(0+tamanho/2-(qy2/7))
        pointsListQuadradox.append(0-tamanho/2)
for q4 in range (0, tamanho*7):
    pointsListQuadradox.append((0-tamanho/2))
    pointsListQuadradoy.append(0-tamanho/2)



while True: #Loop para a conexão com o Arduino
    try:  #Tenta se conectar, se conseguir, o loop se encerra
        arduino = serial.Serial('COM3',115200)
        print('Arduino conectado')
        break
    except:

        pass
angulomax= 12
Amax_realimentacao=12

def mostra_vetores():
    #mostrar vetores salvos
    print("\n____________________________________________")
    print("posiçãox____________________________________________")
    print(vetor_posicao1)
    print("posiçãoy____________________________________________")
    print(vetor_posicao2)
    print("anguloxMPU____________________________________________")
    print(vetor_angulo_x)
    print("anguloyMPU____________________________________________")
    print(vetor_angulo_y)
    print("anguloxReferencia____________________________________________")
    print(vetor_motor_x)
    print("mesayreferencia____________________________________________")
    print(vetor_motor_y)
    print("referenciax____________________________________________")
    print(vetor_setpoint_x)                        
    print("referenciay____________________________________________")
    print(vetor_setpoint_y)        
    print("tempo____________________________________________")
    print(vetor_tempo)

    print("velocidadeAx___________________________________________")
    print(vetor_velocidade_Ax)
    print("velocidadeAy___________________________________________")
    print(vetor_velocidade_Ay)
    print("velocidadeBx___________________________________________")
    print(vetor_velocidade_Bx)
    print("velocidadeBy___________________________________________")
    print(vetor_velocidade_By)
    print("____________________________________________")
    
    
def metros_p_pixel(metor1,metor2):
    pixelx=int((metor1*21)+317.5)
    pixely=int((metor2*21)+317.5)
    return(pixelx,pixely)

def pixel_p_metro(pixel1,pixel2):
    metro1=np.around((pixel1-317.5)/21,2)#21.6
    metro2=np.around((pixel2-317.5)/21,2)
    
    return(metro1,metro2)

def angulomesa_p_angulomotor(mesa1,mesa2):
    '''if mesa2>=angulomax:
        mesa2=angulomax
    if mesa2<=-angulomax:
        mesa2=-angulomax

    if mesa1>=angulomax:
        mesa1=angulomax
    if mesa1<=-angulomax:
        mesa1=-angulomax'''
    mesa1=math.radians(mesa1)
    mesa2=math.radians(mesa2)
    angulo_m1=(math.degrees(math.asin((eixo_mesa/eixo_motor)*math.sin(mesa1))))
    angulo_m2=(math.degrees(math.asin((eixo_mesa/eixo_motor)*math.sin(mesa2))))
        
    return(angulo_m1,angulo_m2)
      
def fecha_malha(posicao,setpoint):
    erro=setpoint-posicao
    #mesa=multiplicador_mesa*erro
    erro=math.degrees(erro)
    #motor=90 #teste
    return (erro)
    
def salva_dados(posicao1,posicao2,angulo1,angulo2,setpoint1,setpoint2,motor1,motor2,tempo,velocidade1,velocidade2,velocidade3,velocidade4):
    vetor_posicao1.append(posicao1)
    vetor_posicao2.append(posicao2)
    vetor_angulo_x.append(angulo1)
    vetor_angulo_y.append(angulo2)
    vetor_setpoint_x.append(setpoint1)
    vetor_setpoint_y.append(setpoint2) 
    vetor_motor_x.append(motor1)
    vetor_motor_y.append(motor2)
    vetor_tempo.append(tempo)
    vetor_velocidade_Ax.append(velocidade1)
    vetor_velocidade_Ay.append(velocidade2)
    vetor_velocidade_Bx.append(velocidade3)
    vetor_velocidade_By.append(velocidade4)

def calcula_velocidade(n00,n01,t00,t01,variacao_inicial):
    velocidade=(n01-n00)/(t01-t00)
    if (variacao_inicial==True):
        velocidade=0
    return velocidade
    

def lê_serial():
    global valores
    while True:
        global dt,Ax,Ay,Az,dt_ant
        arduino.flush()
        #arduino.write((str(motor1) + ":" + str(motor2) + "$").encode())
        msg =str (arduino.readline()) #Lê os dados em formato de string
        msg = msg[2:-5] #Fatia a string
        valores= msg.split(',')
        '''if valores1[0]=='null' or valores1[1]=='null':
            pass
        else:
            valores=valores1'''
        #return #valores

        
def escreve_serial(motor1,motor2):
    arduino.flush()
    arduino.write((str(motor1) + ":" + str(motor2) + "$").encode())

'''def controle_motor_AvAt(angulo,setpoint):
    global motor_x,fps,motor_y,x00,x01,x02,b0,b01,b02,a01,a02
    e=setpoint-angulo
    x00=e+(x01*(-a01))+(x02*a02)
    xresposta=kc*((x00*b0)+(b01*x01)+(b02*x02))
    x02=x01
    x01=x00
    return (xresposta)
'''

##################################################################
#3 variaveis observadas
'''def ordem_reduzida(posicao,referencia,controle0): 
    global xh,G,Amo,Bo,Bm,Aom,Amm,posicao0,controle
    controle0=math.radians(controle0)
    posicao=posicao
    xh=np.dot((Aoo-(G*Amo)),xh)+((Bo-(G*Bm))*controle0)+(G*posicao)+((Aom-(G*Amm))*posicao0)
    posicao0=posicao
    
    e1=(referencia-posicao)*k1
    e2=-(xh[0])*k2
    e3=-xh[1]*k3
    e4=-xh[2]*k4
    controle=e1-e2-e3-e4
    if controle>=0.0872665:
            controle=0.0872665
    if controle<=-0.0872665:
            controle=-0.0872665
    controle=math.degrees(controle)

    return(controle)'''
###################################################################
#2 variaveis observadas
def ordem_reduzida2(posicao2 ,referencia2,angulo2,posicao1,referencia1,angulo1,
                    ukx,uky,posicao00,angulo00,posicao001,angulo001,vbx,vax,vby,vay):
    global xh2,G,Amo,Bo,Bm,Aom,Amm,posicao0,controle,angulo0,posicao01,controle1,angulo01,xh1
    controle2=math.radians(ukx)
    controle1=math.radians(uky)
    xh2=np.array([[vbx],[vay]])
    xh1=np.array([[vby],[vay]])

    angulo2=math.radians(angulo2)
    angulo1=math.radians(-angulo1)
    
    xh2=np.dot((Aoo-np.dot(G,Amo)),xh2)+((Bo-np.dot(G,Bm))*controle2)+np.dot(G,[[posicao2],[angulo2]])+np.dot((Aom-np.dot(G,Amm)),[[posicao00],[angulo00]])
    xh1=np.dot((Aoo-np.dot(G,Amo)),xh1)+((Bo-np.dot(G,Bm))*controle1)+np.dot(G,[[posicao1],[angulo1]])+np.dot((Aom-np.dot(G,Amm)),[[posicao001],[angulo001]])
    
    print(xh2,xh1)
    posicao0=posicao2
    angulo0=angulo2
    posicao01=posicao1
    angulo01=angulo1
    
    e1=(referencia2-posicao2)*k1
    e2=xh2[0]*k2
    e3=angulo2*k3
    e4=xh2[1]*k4
    controle2=(e1-e2-e3-e4)

    e11=(referencia1-posicao1)*k1#*0.7
    e21=(xh1[0])*k2
    e31=angulo1*k3
    e41=xh1[1]*k4
    controle1=(e11-e21-e31-e41)
    
    if controle2>=angulomax:
            controle2=angulomax
    if controle2<=-angulomax:
            controle2=-angulomax
    controle2=math.degrees(controle2)

    if controle1>=angulomax:
            controle1=angulomax
    if controle1<=-angulomax:
            controle1=-angulomax
    controle1=math.degrees(controle1)

    return(controle2,controle1,posicao2,angulo0,posicao01,angulo01)


def Circulo():
    global ix,pointsListCirclex,pointsListCircley,velocidade,spx,spy
    if ix<359:
        spx=pointsListCirclex[ix]
        spy=pointsListCircley[ix]
        ix=ix+1+velocidade*2
                

    else:
        ix=0

    return(spx,spy)

def Quadrado():
    global ix1,tamanho,pointsListQuadradox,pointsListQuadradoy,velocidade,spx,spy
    if ix1<((tamanho*7*4)+ (tamanho*4*7)):
        spx=pointsListQuadradox[ix1]
        spy=pointsListQuadradoy[ix1]
        ix1=ix1+1+velocidade

     
    else:
        ix1=0

    return(spx,spy)
    
    

#inicia leitura da serial
t1 = threading.Thread(target=lê_serial)
t1.start()
time.sleep(1)
escreve_serial(motor2,motor1)
time.sleep(1)
#tempo inicial
dt_ant=time.time()
#offset angulos
#ajuste_ax=0
#ajuste_ay=0
while True: #MEDE ANGULO DA MESA  
    valores_serial=valores
    angulo_x,angulo_y=float(valores_serial[0]), float(valores_serial[1])
    angulox_1=angulo_x
    escreve_serial(motor2,motor1)
    time.sleep(0.07)
    ajuste_ax=0#-angulo_x
    ajuste_ay=0#-angulo_y
    print(angulo_x+ajuste_ax,angulo_y+ajuste_ay)
    if keyboard.is_pressed("q"):
        break
#ZERA MESA    
'''m1,m2=angulomesa_p_angulomotor((-angulo_x+ajuste_ax),(-angulo_y+ajuste_ay))
motor1=motor1-m1
motor2=motor2+m2
escreve_serial(motor2,motor1)
print(motor2,motor1)
time.sleep(1)

while True: #zera mesa --> confirmação
    valores_serial=valores
    angulo_x=float(valores_serial[0])
    angulo_y=float(valores_serial[1])
    print((-angulo_x+ajuste_ax),(-angulo_y+ajuste_ay))
    motory=motor1
    motorx=motor2
    if keyboard.is_pressed("q"):
        break
'''    
#offset angulos --> atualização
#ajuste_ax=0
#ajuste_ay=0

#atualiza parametros do motor


#usando matlab mobile para conferir angulos
while testedeangulos==True:
    while True:
        m1,m2=angulomesa_p_angulomotor((0),(4.5))
        motor1=motory+m1
        motor2=motorx+m2
        escreve_serial(int(motor2),int(motor1))
        t0=time.time()
        valores_serial=valores
        angulo_x,angulo_y=float(valores_serial[0]), float(valores_serial[1])
        #salva dados
        salva_dados(0,0,angulo_x,angulo_y,4.5,0,0,0,time.time(),0,0,0,0)
        while(time.time()-t0<=fps):
            #valores_serial=lê_serial(motor2,motor1)
            """msg = str(arduino.readline())
            arduino.flush()"""
            print('rdy')
        if keyboard.is_pressed("s"):
            testedeangulos=False
            break
print(motor2)      
mostra_vetores()
#tempo inicial --> atualiza
dt_ant=time.time()
escreve_serial(motor2,motor1)

#salva angulos iniciais
angulo_x0=-angulo_x
angulo_y0=-angulo_y
ajuste_ax=0
ajuste_ay=0
#variaveis inicias para calculode velocidade
Ax0=0
Ay0=0
Bx0=0
By0=0
tempo_velocidade=0


def Camera():
    global centro_da_bola,setpoint_xp,setpoint_yp
    while True:
        #detecta bola
        ret, cap2 = cap.read()
        cap2 = cap2[48:692 ,338:982]
        frame = cv2.GaussianBlur(cap2, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, bolaLower, bolaUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
                
        result = cv2.bitwise_and(frame, frame, mask = mask)
        mask1 = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        _,mask1 = cv2.threshold(mask1, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        cnts,hierarchy = cv2.findContours(mask1.copy(),cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"]>0:
                center = (int(M["m10"] / M["m00"]),int(M["m01"] / M["m00"]))
                centro_da_bola=pixel_p_metro(center[0],center[1])
                #print(centro_da_bola)
                if radius > 10:
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                        (237, 28, 36), 2)



        setpoint_xp,setpoint_yp=metros_p_pixel(setpoint_x,setpoint_y)    
        cv2.circle(frame, (int(setpoint_xp), int(setpoint_yp)), int(1),
                            (0, 0, 255), 8)

        
        cv2.imshow('TCC', frame)
        
        if cv2.waitKey(1) & 0xff == 27 :
            cap.release()
            cv2.destroyAllWindows()
            #mostrar vetores salvos
            #mostra_vetores()

            break

        #return
    
t2 = threading.Thread(target=Camera)
t2.start()

        
time.sleep(1)
#posição inicial
centrodabola00=centro_da_bola[0]
centrodabola01=centro_da_bola[1]
posicao01=centro_da_bola[1]/100
posicao0=centro_da_bola[0]/100

time.sleep(1)
def main():
    while True:
        global motor_x,fps,motor_y,x00,x01,x02,b0,b01,b02,a01,a02,setpoint_x,setpoint_y
        global centro_da_bola,motor1,motor2,motorx,motory,dt,dt_ant,cap2,t0
        global angulo_x0,angulo_y0,Ax0,Ay0,Bx0,By0,tempo_velocidade,variacao_inicial
        global mesa1,mesa2,velocidadeAx0,velocidadeAy0,velocidadeBx0,velocidadeBy0
        global centrodabola00,centrodabola01,circulo,quadrado,ix,ix1,ukx,uky,vbx,vax,vby,vay
        global posicao0,angulo0,posicao01,angulo01
        valores_serial=valores
        angulo_x,angulo_y,velocidadeAx,velocidadeAy=float(valores_serial[0]), float(valores_serial[1]),float(valores_serial[2]),float(valores_serial[3])


        #calcula velocidades
        velocidadeBx=calcula_velocidade(Bx0,centro_da_bola[0]/100,tempo_velocidade,time.time(),variacao_inicial)
        velocidadeBy=calcula_velocidade(By0,centro_da_bola[1]/100,tempo_velocidade,time.time(),variacao_inicial)
        tempo_velocidade=time.time()
        variacao_inicial=False
        Bx0=centro_da_bola[0]/100
        By0=centro_da_bola[1]/100

        if circulo==True:
            setpoint_x,setpoint_y=Circulo()
        if quadrado==True:
            setpoint_x,setpoint_y=Quadrado()
        
        
        #fecha malha
        '''mesa2=fecha_malha(Bx0,setpoint_x)
        mesa1=fecha_malha(By0,setpoint_y)'''
       

        #AvAt
        '''mesa2=controle_motor_AvAt(centro_da_bola[0],setpoint_x)
        mesa1=controle_motor_AvAt(centro_da_bola[1],setpoint_y)'''
        

        #realimentação de estados
        '''mesa2=realimentacao_estados(setpoint_x,Bx0,angulo_x,velocidadeBx,velocidadeAx)
        mesa1=realimentacao_estados(setpoint_y,By0,angulo_y,velocidadeBy,velocidadeAy)
        '''
        #observador de ordem reduzida
        mesa2,mesa1,posicao0,angulo0,posicao01,angulo01=ordem_reduzida2(centro_da_bola[0]/100,
                                                                         setpoint_x/100,angulo_x+ajuste_ax,
                                                                         centro_da_bola[1]/100,
                                                                         setpoint_y/100,+ajuste_ay,ukx,uky,
                                                                         posicao0,angulo0,
                                                                         posicao01,angulo01,vbx,vax,vby,vay)

        ukx=mesa2
        uky=mesa1
        

        
        m1,m2=angulomesa_p_angulomotor((mesa1+angulo_y0),(mesa2-angulo_x0))
        motor1=motory +(m1) 
        motor2=motorx+(m2)
        escreve_serial(motor2,motor1)

        vax= float(xh2[1])
        vbx=float(xh2[0])
        vay= float(xh1[1])
        vby=float(xh1[0])

        salva_dados(centro_da_bola[0]/100,centro_da_bola[1]/100,
                    angulo_x+ajuste_ax,angulo_y+ajuste_ay,setpoint_x/100,
                    setpoint_y/100,mesa2,mesa1,(time.time()),vbx,vby,vax,vay)
        
        #tempo controle
        '''while((time.perf_counter()-t0)-fps<0):
            print(mesa2)'''
        t01=time.perf_counter()-t0
        
        if t01<=fps:
                time.sleep((fps-t01))
        
        
        t0=time.perf_counter()
        
        if keyboard.is_pressed("q"):
            mostra_vetores()
            cap.release()
            #cv2.destroyAllWindows()
            #mostrar vetores salvos
            

            break




t0=time.perf_counter()

t3 = threading.Thread(target=main)
t3.start()


