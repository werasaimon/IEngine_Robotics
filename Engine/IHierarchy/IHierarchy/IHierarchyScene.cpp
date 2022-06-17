#include "IHierarchyScene.h"
#include <algorithm>

namespace IEngine
{



// Global list of active scenes
static std::vector<IHierarchyScene*> __sceneList;


IHierarchyScene::IHierarchyScene()
    :  _id(""), _firstNode(nullptr), _lastNode(nullptr), _nodeCount(0), //_bindAudioListenerToCamera(true),
      _nextItr(nullptr), _nextReset(true)
{
   __sceneList.push_back(this);
}

IHierarchyScene::~IHierarchyScene()
{
    // Remove all nodes from the scene
    RemoveAllNodes();

    // Remove the scene from global list
    std::vector<IHierarchyScene*>::iterator itr = std::find(__sceneList.begin(), __sceneList.end(), this);
    if (itr != __sceneList.end())
        __sceneList.erase(itr);
}


IHierarchyScene *IHierarchyScene::Create(const char *id)
{
    IHierarchyScene* scene = new IHierarchyScene();
    scene->SetId(id);
    return scene;
}


IHierarchyScene *IHierarchyScene::GetScene(const char *id)
{
    if (id == nullptr)
        return __sceneList.size() ? __sceneList[0] : nullptr;

    for (size_t i = 0, count = __sceneList.size(); i < count; ++i)
    {
        if (__sceneList[i]->_id == id)
            return __sceneList[i];
    }

    return nullptr;
}

const char *IHierarchyScene::GetId() const
{
  return _id.c_str();
}

void IHierarchyScene::SetId(const char *id)
{
   _id = id ? id : "";
}

IHierarchyNode *IHierarchyScene::AddNode(unsigned int id)
{
    IHierarchyNode* node = IHierarchyNode::Create(id);
    assert(node);
    AddNode(node);

    // Call release to decrement the ref count to 1 before returning.
    //node->release();

    return node;
}

void IHierarchyScene::AddNode(IHierarchyNode *node)
{
    assert(node);

    if (node->_scene == this)
    {
        // The node is already a member of this scene.
        return;
    }

    //node->addRef();

    // If the node is part of another scene, remove it.
    if (node->_scene && node->_scene != this)
    {
        node->_scene->RemoveNode(node);
    }

    // If the node is part of another node hierarchy, remove it.
    if (node->GetParent())
    {
        node->GetParent()->RemoveChild(node);
    }

    // Link the new node into the end of our list.
    if (_lastNode)
    {
        _lastNode->_nextSibling = node;
        node->_prevSibling = _lastNode;
        _lastNode = node;
    }
    else
    {
        _firstNode = _lastNode = node;
    }

    node->_scene = this;

    ++_nodeCount;

//    // If we don't have an active camera set, then check for one and set it.
//    if (_activeCamera == NULL)
//    {
//        Camera* camera = node->getCamera();
//        if (camera)
//        {
//            setActiveCamera(camera);
//        }
//    }
}

void IHierarchyScene::RemoveNode(IHierarchyNode *node)
{
    assert(node);

    if (node->_scene != this)
        return;

    if (node == _firstNode)
    {
        _firstNode = node->_nextSibling;
    }
    if (node == _lastNode)
    {
        _lastNode = node->_prevSibling;
    }

    node->remove();
    node->_scene = nullptr;

    // SAFE_RELEASE(node);

    --_nodeCount;
}

void IHierarchyScene::RemoveAllNodes()
{
    while (_lastNode)
    {
        RemoveNode(_lastNode);
    }
}


unsigned int IHierarchyScene::GetNodeCount() const
{
    return _nodeCount;
}

IHierarchyNode* IHierarchyScene::GetFirstNode() const
{
    return _firstNode;
}


void IHierarchyScene::Update(float elapsedTime)
{
    for (IHierarchyNode* node = _firstNode; node != nullptr; node = node->_nextSibling)
    {
//        if (node->isEnabled())
            node->Update(elapsedTime);
    }
}



void IHierarchyScene::Reset()
{
    _nextItr = nullptr;
    _nextReset = true;
}


}

