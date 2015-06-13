/*
	bola.cpp 
	Miguel Leitao, ISEP, 2008
*/


#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <btBulletDynamicsCommon.h>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>


int ResetFlag=0,PosFlag=0, CamFlag=1, flag=0, flag1=0, ComandFlag=0;
int mouse=0,count=0, count1=0, p=0;
float altura = 2.;
double X=0, Y=0;
double z_bola = 7.;


btRigidBody* ballRigidBody;
btRigidBody* pinoRigidBody[10];
btRigidBody* pranchaRigidBody = NULL;

//apontadores para variaveis globais
osgViewer::Viewer *pointer_viewer;
osg::Matrix *point_myMatrix;
osg::Matrix *pointer_camMatrix;



osg::Vec3 bt2osg_Vec3(btVector3 bv) {
	return osg::Vec3( bv.x(), bv.y(), bv.z() );
	}
osg::Vec4 bt2osg_Vec4(btVector4 bv) {
	return osg::Vec4( bv.x(), bv.y(), bv.z(), bv.w() );
	}
btVector3 osg2bt_Vec3(osg::Vec3 bv) {
	return btVector3( bv.x(), bv.y(), bv.z() );
	}
osg::Quat bt2osg_Quat(btQuaternion bv) {
	return osg::Quat( bv.x(), bv.y(), bv.z(), bv.w() );
	}

void move(void); //prototipo da função de mover a base com as teclas
void Reset(void); //função reset ao jogo
void Camera(void); //função para selecionar as cameras 1 2 3 (4 mal)
void som(void); //função para chamar o som

class EventHandler : public osgGA::GUIEventHandler
		{
		public:
			bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
			{
			 osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
			 if (!viewer) return false;

			 switch(ea.getEventType())
			  {
				case(osgGA::GUIEventAdapter::KEYUP):
					switch ( ea.getKey() ) {

						case 'r':		//reset
							flag = 1;
							ResetFlag = 6;
							break;

						case 'w':		//mover prancha
							PosFlag = 1;
							break;
						case 's':
							PosFlag = 2;
							break;
						case 'a':
							PosFlag = 3;
							break;
						case 'd':
							PosFlag = 4;
							break;



						case 'q':		//encerrar
							flag1 = 1;
							break;




						case '1':		//modo de camera
							CamFlag=1;
							break;
						case '2':
							CamFlag=2;
							break;
						case '3':
							CamFlag=3;
							break;
						case '4':
							CamFlag=4;
							break;



						//comandos de prancha

						case '5':		
							ComandFlag=1;	//rato
							break;
						case '6':		//teclado apenas 2 eixos
							ComandFlag=0;
							break;
						}
					
		
				case(osgGA::GUIEventAdapter::MOVE):
						if(ComandFlag==1){
						//std::cout << "mouse move"<< ea.getX()<< " " << ea.getY()<< std::endl;
					
						if ( pranchaRigidBody ) {
							btTransform trans;
							pranchaRigidBody->getMotionState()->getWorldTransform(trans);
							trans.setRotation(btQuaternion((double)(ea.getX()-500.)/1000.,-(double)(ea.getY()-300.)/1000., 0.) ); //controla prancha com o rato
							pranchaRigidBody->getMotionState()->setWorldTransform(trans);
						}
						}
						return false;

				default:
						return false;
			  }
			}
		};



	
	
osg::MatrixTransform *AddBox(osg::Group* upNode, btCompoundShape* upShape, float sx, float sy, float sz, float x0, float y0, float z0)
	{
		//osg::Matrix myMatrix;
		osg::Node* loadedModel = osgDB::readNodeFile("cube.obj");

		// Graphical Node
		osg::MatrixTransform* myTransform = new osg::MatrixTransform;
		*point_myMatrix = osg::Matrix::scale(sx/2., sy/2., sz/2.);
		point_myMatrix->setTrans( x0, y0, z0);
		myTransform->setMatrix( *point_myMatrix );
		myTransform->addChild(loadedModel);
		upNode->addChild(myTransform);

		// Collision shape
		btCollisionShape* boxShape = new btBoxShape(btVector3(sx/2.,sy/2.,sz/2.));
		upShape->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(x0,y0,z0)), boxShape );

		return myTransform;
	}
	








int main()
{
	
	int zi=5;


	// Create dynamic world
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
	


	// Set Gravity
	dynamicsWorld->setGravity(btVector3(0.,0.,-9.8));
	osg::Matrix myMatrix;
	point_myMatrix=&myMatrix;


	//estrutura   nó principal -> nó sombras -> nó secundário e luzes
	// Creating the root node
        osg::Group* SceneRoot = new osg::Group; // nó root secundário
	osg::Group* SceneROOT = new osg::Group; //nó root 
	//SceneROOT->addChild( SceneRoot );

	// Creating the viewer
	osgViewer::Viewer viewer;
	pointer_viewer=&viewer;
	viewer.setSceneData( SceneROOT );

	

	// add the Event handler
	  viewer.addEventHandler(new EventHandler());


	// Add Light Source
	osg::LightSource *ls = new osg::LightSource;
	ls->getLight()->setPosition(osg::Vec4(0.5,-0.7,1.,0.));
	ls->getLight()->setAmbient(osg::Vec4(0.2,0.2,0.2,1));
	ls->getLight()->setDiffuse(osg::Vec4(0.6,0.6,0.6,1.0));
	//SceneRoot->addChild( ls );

	//Shadow stuff!!!	
	osgShadow::ShadowedScene *shadowScene = new osgShadow::ShadowedScene;
	osgShadow::ShadowMap *sm = new osgShadow::ShadowMap;
      	shadowScene->setShadowTechnique(sm);
	
	shadowScene->addChild(SceneRoot);
	shadowScene->addChild(ls);
	SceneROOT->addChild( shadowScene );
	

	// add the shadowed scene to our root (the objects need to be added to the shadowed scene, NOT the root node)





//Bola///////////////////////////////////////////////////////////////////////////////////
        osg::Node* loadedModel = osgDB::readNodeFile("bola.obj");	
	osg::Matrix bolaMatrix;
	osg::MatrixTransform* bolaPos = new osg::MatrixTransform;
	
	bolaPos->setMatrix( bolaMatrix );
	bolaPos->addChild( loadedModel );
	
	
	//bolaPos->addChild(loadedModel);
	bolaMatrix.setTrans( -14.,4.,z_bola);
	
	//bolaMatrix.makeScale(0.9,0.9,0.9);
	bolaPos->setMatrix( bolaMatrix );

	
	SceneRoot->addChild(bolaPos);

	btCollisionShape* ballShape = new btSphereShape(0.5);
	btDefaultMotionState* ballMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(-14.,4.,z_bola)));
	btScalar mass = 7.3;
	btVector3 ballInertia(0,0,0);
	ballShape->calculateLocalInertia(mass,ballInertia);
	btRigidBody::btRigidBodyConstructionInfo
	
		ballRigidBodyCI(mass,ballMotionState,ballShape,ballInertia);
		ballRigidBodyCI.m_restitution = 0.f;
	ballRigidBodyCI.m_friction = 0.6f;
	ballRigidBody = new btRigidBody(ballRigidBodyCI);
	dynamicsWorld->addRigidBody(ballRigidBody);
	






////////// pino////////////////////////////////////////////////////////////////////////////
	// 10 pinos
	float raio = 0.35;
	
	mass = 1.6;
	loadedModel = osgDB::readNodeFile("pino.obj");
	btVector3 pinoInertia(0,0,0);
	

	
	btCompoundShape* pinoShape = new btCompoundShape();
	pinoShape->
		addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)), new btCylinderShapeZ(btVector3(.2,raio,altura/2)) );
	pinoShape->
		addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,-0.1)), new btSphereShape(0.36) );
	pinoShape->
		addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0.75)), new btSphereShape(0.25) );


	pinoShape->calculateLocalInertia(mass,pinoInertia);
	osg::MatrixTransform* pinoPos[10];

	int p;
	int l=1, ln=1, lp=0;
	float px=1., py;
	for( p=0 ; p<10 ; p++ ) {

			pinoPos[p] = new osg::MatrixTransform;
			pinoPos[p]->addChild(loadedModel);
			SceneRoot->addChild(pinoPos[p]);
			lp++;
			if ( lp>ln ) { l++; lp=1; ln++; px=(float)l; }
			py = -0.5 + (float)lp - ((float)l)/2.;
			btDefaultMotionState* pinoMotionState = new
				btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(px,py,altura/2.)));

			btRigidBody::btRigidBodyConstructionInfo
			pinoRigidBodyCI(mass,pinoMotionState,pinoShape,pinoInertia);
			pinoRigidBodyCI.m_restitution = 0.95f;
			pinoRigidBodyCI.m_friction = 0.8f;
			pinoRigidBody[p] = new btRigidBody(pinoRigidBodyCI);
			dynamicsWorld->addRigidBody(pinoRigidBody[p]);

	}





/////////Ground////////////////////////////////////////////////////////////////////////////////////////
	osg::MatrixTransform* myTransform = new osg::MatrixTransform;
	myMatrix = osg::Matrix::rotate( 0, osg::Vec3(0.,1.,0.) );
	osg::Vec3 myNorm = myMatrix.preMult( osg::Vec3(0.,0.,1.)) ;
	double z0 = 0;
	myMatrix.setTrans( 0., 0, z0);
	myTransform->setMatrix( myMatrix );
	loadedModel = osgDB::readNodeFile("plano.obj");
	myTransform->addChild(loadedModel);
	SceneRoot->addChild(myTransform);


	btCollisionShape* groundShape = new btStaticPlaneShape(osg2bt_Vec3(myNorm), z0);
	btDefaultMotionState* groundMotionState = new 	btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
	btRigidBody::btRigidBodyConstructionInfo
		groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	groundRigidBodyCI.m_restitution = 0.2f;
	groundRigidBodyCI.m_friction = 0.2f;
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	dynamicsWorld->addRigidBody(groundRigidBody);


	
	//Ground2
	osg::MatrixTransform* myTransform2 = new osg::MatrixTransform;
	myMatrix = osg::Matrix::rotate( 0.20, osg::Vec3(0.,1.,0.) );
	osg::Vec3 myNorm2 = myMatrix.preMult( osg::Vec3(0.,0.,1.) ) ;
	myMatrix.setTrans( -5., 0., 0.);
	myTransform2->setMatrix( myMatrix );
	loadedModel = osgDB::readNodeFile("plano.obj");
	myTransform2->addChild(loadedModel);
	SceneRoot->addChild(myTransform2);


	
	//ground 2 dinamic
	btCollisionShape* groundShape2 = new btStaticPlaneShape(osg2bt_Vec3(myNorm2), z0);
	btDefaultMotionState* groundMotionState2 = new 	btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(-5.,0,0)));
	btRigidBody::btRigidBodyConstructionInfo
		groundRigidBodyCI2(0,groundMotionState2,groundShape2,btVector3(0,0,0));
	groundRigidBodyCI2.m_restitution = 0.95f;
	groundRigidBodyCI2.m_friction = 0.2f;
	btRigidBody* groundRigidBody2 = new btRigidBody(groundRigidBodyCI2);
	dynamicsWorld->addRigidBody(groundRigidBody2);






////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	

	
	
	// Prancha
	osg::MatrixTransform* pranchaTransform = new osg::MatrixTransform;
	btCompoundShape* pranchaShape = new btCompoundShape();
	AddBox(pranchaTransform, pranchaShape, 10., 10., 0.1, -5., 0., 1.+zi);
	SceneRoot->addChild(pranchaTransform);

	btDefaultMotionState* pranchaMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(-10.,0.,0.)));

	btRigidBody::btRigidBodyConstructionInfo pranchaRigidBodyCI(0.,pranchaMotionState,pranchaShape,btVector3(0.,0.,0.));

	pranchaRigidBodyCI.m_restitution = 0.f;
	pranchaRigidBodyCI.m_friction = 0.8f;

	pranchaRigidBody = new btRigidBody(pranchaRigidBodyCI);
	pranchaRigidBody->setCollisionFlags( pranchaRigidBody->getCollisionFlags() |
		btCollisionObject::CF_KINEMATIC_OBJECT );
	pranchaRigidBody->setActivationState(DISABLE_DEACTIVATION);
	dynamicsWorld->addRigidBody(pranchaRigidBody);
	
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	float zs=1;	
	//limites laterais
	AddBox(pranchaTransform, pranchaShape, 10., 0.1, 1., -5., 5., 1.5+zi);//limite direito
	AddBox(pranchaTransform, pranchaShape, 10., 0.1, 1., -5., -5., 1.5+zi);//limite esquerdo
	AddBox(pranchaTransform, pranchaShape, 0.1, 10., 1., -10., 0, 1.5+zi);//limite superior
	AddBox(pranchaTransform, pranchaShape, 0.1, 4.0, 1., 0., -3.0, 1.5+zi); //limite inferior esquerdo
	AddBox(pranchaTransform, pranchaShape, 0.1, 4.0, 1., 0., 3.0, 1.5+zi); //limte inferior direito
	

	//barras +-
	AddBox(pranchaTransform, pranchaShape, zs*0.5, zs*8.0, 1., -9.0, 1., 1.5+zi);//c14
	AddBox(pranchaTransform, pranchaShape, zs*0.5, zs*8.0, 1., -7.0,-1., 1.5+zi);//c14
	AddBox(pranchaTransform, pranchaShape, zs*0.5, zs*8.0, 1., -5.0,  1., 1.5+zi);//c14
	AddBox(pranchaTransform, pranchaShape, zs*0.5, zs*8.0, 1., -3.0, -1., 1.5+zi);//c14
	



	
	btRigidBody* boxRigidBody;

	// Box - Limitadora da pista bowling
	osg::MatrixTransform* boxTransform = new osg::MatrixTransform;
	btCompoundShape* boxShape = new btCompoundShape();
	//SceneRoot->addChild(boxTransform);
	btDefaultMotionState* boxMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
	btRigidBody::btRigidBodyConstructionInfo boxRigidBodyCI(0.,boxMotionState,boxShape,btVector3(0,0,0));
	boxRigidBodyCI.m_restitution = 0.f;
	boxRigidBodyCI.m_friction = 0.2f;
	boxRigidBody = new btRigidBody(boxRigidBodyCI);
	dynamicsWorld->addRigidBody(boxRigidBody);
	AddBox(boxTransform, boxShape, 20., 1., 500., 0., 10., 0.); //Esquerda
	AddBox(boxTransform, boxShape, 1., 20., 500., 10, 0., 0.); //Frente
	AddBox(boxTransform, boxShape, 1., 20., 500., -30, 0., 0.); //Traseira
	AddBox(boxTransform, boxShape, 20., 1., 500., 0., -10., 0.); //Direita



	
	// Setup camera
	osg::Matrix camMatrix;
	pointer_camMatrix=&camMatrix;
	//

	// record the timer tick at the start of rendering.
	osg::Timer myTimer;
	double time_now = myTimer.time_s();
    	double last_time = time_now;
	double frame_time;
	double m=0, k=0.9;
    	btTransform trans;
	osg::Matrix matrix;
	
 while( !viewer.done() )
  	{
	 
	


     	  viewer.frame();
	  time_now = myTimer.time_s();
	  frame_time = time_now - last_time;
	  last_time = time_now;
	
	dynamicsWorld->stepSimulation(frame_time, 10); 

	
	ballRigidBody->getMotionState()->getWorldTransform(trans);
	bolaMatrix.makeRotate(bt2osg_Quat(trans.getRotation()));
	bolaMatrix.setTrans(bt2osg_Vec3(trans.getOrigin()));
	bolaPos->setMatrix(bolaMatrix);

	for( p=0 ; p<10 ; p++ ) {
			
			pinoRigidBody[p]->getMotionState()->getWorldTransform(trans);
			matrix.makeRotate(bt2osg_Quat(trans.getRotation()));
			matrix.setTrans(bt2osg_Vec3(trans.getOrigin()));
			pinoPos[p]->setMatrix(matrix);
			
			
			//contar
			if(flag==1){ //compara alturas dos pinos
   
				m=trans.getOrigin().getZ();
				if (m <=k ){count++;};
				if (count==10){som();};
				if (p==9){flag=0;};
			}
	
	}
	
	
	move();
	pranchaRigidBody->getMotionState()->getWorldTransform(trans);
	myMatrix.makeRotate(bt2osg_Quat(trans.getRotation()));
	myMatrix.setTrans(bt2osg_Vec3(trans.getOrigin()));
	pranchaTransform->setMatrix(myMatrix);
	Reset();
	Camera ();
	count1=count1+count;
	count=0;
	if(flag1==1){
		printf("Pontuação = %d \n", count1);
			exit(0);};
  	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





void move(){ /////mover a base com teclas
	if (PosFlag==1){X=X+0.01;}

	if (PosFlag==2){X=X-0.01;}

	if (PosFlag==3){Y=Y+0.01;}

	if (PosFlag==4){Y=Y-0.01;}


	if ( ballRigidBody && PosFlag!=0 && ComandFlag==0 ) {
						btTransform trans;
						pranchaRigidBody->getMotionState()->getWorldTransform(trans);
						trans.setRotation(btQuaternion(X,Y,0.));
						pranchaRigidBody->getMotionState()->setWorldTransform(trans);
						
					}

	PosFlag=0;

}




void Reset() { ////////recolocar pinos
	


	if (ResetFlag==0) return;
	std::cout << "Reset" << std::endl;
	int l=1, ln=1, lp=0;
	float px=1., py;

	if ( ResetFlag==1 ) {
		ballRigidBody->setCollisionFlags( pinoRigidBody[p]->getCollisionFlags() &! btCollisionObject::CF_KINEMATIC_OBJECT );
		for( p=0 ; p<10 ; p++ )
			pinoRigidBody[p]->setCollisionFlags( pinoRigidBody[p]->getCollisionFlags() &! btCollisionObject::CF_KINEMATIC_OBJECT );
	}
	if ( ResetFlag>=3 ) {
		btTransform trans;
		ballRigidBody->setCollisionFlags( ballRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
		ballRigidBody->clearForces();
		ballRigidBody->setLinearVelocity( btVector3(0,0,0) );
		ballRigidBody->setAngularVelocity( btVector3(0,0,0) );
		ballRigidBody->getMotionState()->getWorldTransform(trans);
		trans.setRotation(btQuaternion(0,0,0,1));
		trans.setOrigin(btVector3(-19.,-4.,z_bola)); // Posicao inicial da bola
		ballRigidBody->getMotionState()->setWorldTransform(trans);
		
		for( p=0 ; p<10 ; p++ ) {
			pinoRigidBody[p]->setCollisionFlags( pinoRigidBody[p]->getCollisionFlags() |	btCollisionObject::CF_KINEMATIC_OBJECT );
			pinoRigidBody[p]->clearForces();
			pinoRigidBody[p]->setLinearVelocity( btVector3(0,0,0) );
			pinoRigidBody[p]->setAngularVelocity( btVector3(0,0,0) );
			pinoRigidBody[p]->getMotionState()->getWorldTransform(trans);
			lp++;
			if ( lp>ln ) { l++; lp=1; ln++; px=(float)l; }
			py = -0.5 + (float)lp - ((float)l)/2.;
			trans.setRotation(btQuaternion(0,0,0,1));
			trans.setOrigin(btVector3(px,py,1.));
			pinoRigidBody[p]->getMotionState()->setWorldTransform(trans);
		}
	}
	ResetFlag--;
	
}




void Camera() ///função para diferentes prespectivas da camera
{	
	
	if(CamFlag==1){
 	pointer_camMatrix->makeLookAt( osg::Vec3(0.,-30.,15.), osg::Vec3(-5.,-5.,5.), osg::Vec3(0.,0.,1.) );
 	pointer_viewer->getCamera()->setViewMatrix(*pointer_camMatrix);
	}
	
	if(CamFlag==2){
 	pointer_camMatrix->makeLookAt( osg::Vec3(30.,-30.,15.), osg::Vec3(-10.,20.,-5.), osg::Vec3(0.,0.,6.) );
 	pointer_viewer->getCamera()->setViewMatrix(*pointer_camMatrix);
	}
	if(CamFlag==3){
 	pointer_camMatrix->makeLookAt( osg::Vec3(40.,0.,30.), osg::Vec3(-20.,0.,-10.), osg::Vec3(0.,0.,5.) );
 	pointer_viewer->getCamera()->setViewMatrix(*pointer_camMatrix);


	}
	if(CamFlag==4){ //camera com o rato entra mas não sai...
 	pointer_viewer->setCameraManipulator(  new osgGA::TrackballManipulator() );
	}
	if(CamFlag!=0){
 	CamFlag=0;
	}
	
}


void som() 
{
	//sound
	

	system("play ./sound.wav");

}

