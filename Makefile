
INC_BULLET=-I/usr/include/bullet
INC_OSG=-I/usr/include/osg
LD_OSG=-l osg -l osgViewer -losgSim -l osgDB -l osgGA -l osgShadow
BASE_DIR=http://ave.dee.isep.ipp.pt/~jml/intmu/lab9


jogo: jogo.o
	cc -O2 -o $@ $^ -l BulletDynamics -l BulletCollision -l LinearMath ${LD_OSG} -l stdc++ -lm

jogo.o: jogo.cpp
	g++ -Wall -c ${INC_BULLET} ${INC_OSG} $<

	
