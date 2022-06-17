#include "IHierarchyNode.h"
#include "IHierarchyScene.h"

namespace IEngine
{


// Node dirty flags
#define NODE_DIRTY_WORLD 1
#define NODE_DIRTY_BOUNDS 2
#define NODE_DIRTY_HIERARCHY 4
#define NODE_DIRTY_ALL (NODE_DIRTY_WORLD | NODE_DIRTY_BOUNDS | NODE_DIRTY_HIERARCHY)

IHierarchyNode::IHierarchyNode(unsigned int id)
    :  _scene(nullptr), _firstChild(nullptr), _nextSibling(nullptr), _prevSibling(nullptr), _parent(nullptr), _childCount(0), _enabled(true), _tags(nullptr),
       _dirtyBits(NODE_DIRTY_ALL)
{
    if (id)
    {
        _id = id;
    }
}


IHierarchyNode *IHierarchyNode::Create(unsigned int id)
{
    return new IHierarchyNode(id);
}

const char *IHierarchyNode::GetTypeName() const
{
    return "IHierarchyNode";
}

unsigned int IHierarchyNode::GetId() const
{
    return _id;
}

void IHierarchyNode::SetId(unsigned int id)
{
    if (id)
    {
        _id = id;
    }
}

IHierarchyNode::Type IHierarchyNode::GetTypeNode() const
{
    return  IHierarchyNode::NODE;
}

void IHierarchyNode::AddChild(IHierarchyNode *child)
{

       assert(child);

       if (child->_parent == this)
       {
           // This node is already present in our hierarchy
           return;
       }
      // child->addRef();

       // If the item belongs to another hierarchy, remove it first.
       if (child->_parent)
       {
           child->_parent->RemoveChild(child);
       }
       else if (child->_scene)
       {
           child->_scene->RemoveNode(child);
       }
       // Add child to the end of the list.
       // NOTE: This is different than the original behavior which inserted nodes
       // into the beginning of the list. Although slightly slower to add to the
       // end of the list, it makes scene traversal and drawing order more
       // predictable, so I've changed it.
       if (_firstChild)
       {
           IHierarchyNode* n = _firstChild;
           while (n->_nextSibling)
           {
               n = n->_nextSibling;
           }
           n->_nextSibling = child;
           child->_prevSibling = n;
       }
       else
       {
           _firstChild = child;
       }
       child->_parent = this;
       ++_childCount;
       //setBoundsDirty();

//       if (_dirtyBits & NODE_DIRTY_HIERARCHY)
//       {
//          // hierarchyChanged();
//       }
}

void IHierarchyNode::RemoveChild(IHierarchyNode *child)
{
    if (child == nullptr || child->_parent != this)
    {
        // The child is not in our hierarchy.
        return;
    }
    // Call remove on the child.
    child->remove();
}

void IHierarchyNode::RemoveAllChildren()
{
    _dirtyBits &= ~NODE_DIRTY_HIERARCHY;
    while (_firstChild)
    {
        RemoveChild(_firstChild);
    }
    _dirtyBits |= NODE_DIRTY_HIERARCHY;
    // hierarchyChanged();
}



void IHierarchyNode::remove()
{
    // Re-link our neighbours.
    if (_prevSibling)
    {
        _prevSibling->_nextSibling = _nextSibling;
    }
    if (_nextSibling)
    {
        _nextSibling->_prevSibling = _prevSibling;
    }
    // Update our parent.
    IHierarchyNode* parent = _parent;
    if (parent)
    {
        if (this == parent->_firstChild)
        {
            parent->_firstChild = _nextSibling;
        }
        --parent->_childCount;
    }
    _nextSibling = nullptr;
    _prevSibling = nullptr;
    _parent = nullptr;

//    if (parent && parent->_dirtyBits & NODE_DIRTY_HIERARCHY)
//    {
//        parent->hierarchyChanged();
    //    }
}


IHierarchyNode* IHierarchyNode::GetFirstChild() const
{
    return _firstChild;
}

IHierarchyNode* IHierarchyNode::GetNextSibling() const
{
    return _nextSibling;
}

IHierarchyNode* IHierarchyNode::GetPreviousSibling() const
{
    return _prevSibling;
}

IHierarchyNode* IHierarchyNode::GetParent() const
{
    return _parent;
}

unsigned int IHierarchyNode::GetChildCount() const
{
    return _childCount;
}

IHierarchyNode* IHierarchyNode::GetRootNode() const
{
    IHierarchyNode* n = const_cast<IHierarchyNode*>(this);
    while (n->GetParent())
    {
        n = n->GetParent();
    }
    return n;
}


bool IHierarchyNode::isEnabled() const
{
    return _enabled;
}

void IHierarchyNode::Setup()
{
    for (IHierarchyNode* child = _firstChild; child != nullptr; child = _nextSibling)
    {
        child->Setup();
    }
}


void IHierarchyNode::Update(float elapsedTime)
{
     for (IHierarchyNode* child = _firstChild; child != nullptr; child = _nextSibling)
     {
         child->Update(elapsedTime);
     }
}
//==========================================================================//


IHierarchyNode *IHierarchyNode::Clone() const
{
   IHierarchyNodeCloneContext context;
   return CloneRecursive(context);
}

void IHierarchyNode::Copy(const IHierarchyNode *node)
{
   _scene = node->_scene;
   _id = node->_id;
   _firstChild = node->_firstChild;
   _nextSibling = node->_nextSibling;
   _prevSibling = node->_prevSibling;
   _parent = node->_parent;
   _childCount = node->_childCount;
   _enabled = node->_enabled;
   _tags = node->_tags;
   _dirtyBits = node->_dirtyBits;
}


IHierarchyNode *IHierarchyNode::CloneSingleNode(IHierarchyNodeCloneContext &context) const
{
    IHierarchyNode* copy = IHierarchyNode::Create(GetId());
    context.registerClonedNode(this, copy);
    CloneInto(copy, context);
    return copy;
}

IHierarchyNode *IHierarchyNode::CloneRecursive(IHierarchyNodeCloneContext &context) const
{
    IHierarchyNode* copy = CloneSingleNode(context);
    assert(copy);

    // Add child nodes
    for ( IHierarchyNode* child = GetFirstChild(); child != NULL; child = child->GetNextSibling())
    {
        IHierarchyNode* childCopy = child->CloneRecursive(context);
        assert(childCopy);
        copy->AddChild(childCopy);
        //childCopy->release();
    }

    return copy;
}

void IHierarchyNode::CloneInto(IHierarchyNode *node, IHierarchyNodeCloneContext &context) const
{
    assert(node);

//      Transform::cloneInto(node, context);

//      if (Drawable* drawable = getDrawable())
//      {
//          Drawable* clone = drawable->clone(context);
//          node->setDrawable(clone);
//          Ref* ref = dynamic_cast<Ref*>(clone);
//          if (ref)
//              ref->release();
//      }
//      if (Camera* camera = getCamera())
//      {
//          Camera* clone = camera->clone(context);
//          node->setCamera(clone);
//          Ref* ref = dynamic_cast<Ref*>(clone);
//          if (ref)
//              ref->release();
//      }
//      if (Light* light = getLight())
//      {
//          Light* clone = light->clone(context);
//          node->setLight(clone);
//          Ref* ref = dynamic_cast<Ref*>(clone);
//          if (ref)
//              ref->release();
//      }
//      if (AudioSource* audio = getAudioSource())
//      {
//          AudioSource* clone = audio->clone(context);
//          node->setAudioSource(clone);
//          Ref* ref = dynamic_cast<Ref*>(clone);
//          if (ref)
//              ref->release();
//      }
//      if (_tags)
//      {
//          node->_tags = new std::map<std::string, std::string>(_tags->begin(), _tags->end());
//      }

//      node->_world = _world;
//      node->_bounds = _bounds;

      // TODO: Clone the rest of the node data.
}




//==========================================================================//

IHierarchyNodeCloneContext::IHierarchyNodeCloneContext()
{
}

IHierarchyNodeCloneContext::~IHierarchyNodeCloneContext()
{
}

IHierarchyNode* IHierarchyNodeCloneContext::findClonedNode(const IHierarchyNode* node)
{
    assert(node);
    std::map<const IHierarchyNode*, IHierarchyNode*>::iterator it = _clonedNodes.find(node);
    return it != _clonedNodes.end() ? it->second : NULL;
}

void IHierarchyNodeCloneContext::registerClonedNode(const IHierarchyNode* original, IHierarchyNode* clone)
{
    assert(original);
    assert(clone);
    _clonedNodes[original] = clone;
}



}
