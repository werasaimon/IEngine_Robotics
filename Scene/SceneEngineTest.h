#ifndef SCENEENGINETEST_H
#define SCENEENGINETEST_H

#include "SceneMain.h"

class SceneEngineTest : public SceneMain
{
public:
    SceneEngineTest();


    void initialization();
    void render(float FrameTime);
    void update();
    void resize( float width , float height );

    void mouseMove( float x , float y  , int button);
    void mousePress( float x , float y , int button );
    void mouseReleasePress( float x , float y , int button );
    void mouseWheel( float delta );

    void keyboard(int key );
    void destroy();
};

#endif // SCENEENGINETEST_H
