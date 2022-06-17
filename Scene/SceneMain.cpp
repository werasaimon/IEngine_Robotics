#include "SceneMain.h"


SceneMain::SceneMain()
{

}

SceneMain::~SceneMain()
{

}

QTextEdit *ConsoleScene::textConsole() const
{
    return mTextConsole;
}

void ConsoleScene::setTextConsole(QTextEdit *textConsole)
{
    mTextConsole = textConsole;
}
