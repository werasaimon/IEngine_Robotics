#ifndef IHIERARCHYNODE_H
#define IHIERARCHYNODE_H

#include <assert.h>
#include <vector>
#include <map>


namespace IEngine
{

class IHierarchyNodeCloneContext;
class IHierarchyScene;

class IHierarchyNode
{
    friend class IHierarchyScene;

   protected:

    /** The scene this node is attached to. */
      IHierarchyScene* _scene;
      /** The nodes id. */
      unsigned int _id;
      /** The nodes first child. */
      IHierarchyNode* _firstChild;
      /** The nodes next sibiling. */
      IHierarchyNode* _nextSibling;
      /** The nodes previous sibiling. */
      IHierarchyNode* _prevSibling;
      /** The nodes parent. */
      IHierarchyNode* _parent;
      /** The number of child nodes. */
      unsigned int _childCount;
      /** If this node is enabled. Maybe different if parent is enabled/disabled. */
      bool _enabled;
      /** Tags assigned to this node. */
      std::map<std::string, std::string>* _tags;


      /** The dirty bits used for optimization. */
      mutable int _dirtyBits;
 public:

     IHierarchyNode();

      //=====================================================//

     /**
       * Defines the types of nodes.
       */
      enum Type
      {
          NODE = 1,
          JOINT
      };

      /**
       * Creates a new node with the specified ID.
       *
       * @param id The ID for the new node.
       * @script{create}
       */
      static IHierarchyNode* Create(unsigned int id);


      /**
        * Extends ScriptTarget::getTypeName() to return the type name of this class.
        *
        * @return The type name of this class: "Node"
        * @see ScriptTarget::getTypeName()
        */
      const char* GetTypeName() const;

      /**
        * Gets the identifier for the node.
        *
        * @return The node identifier.
        */
      unsigned int GetId() const;

      /**
        * Sets the identifier for the node.
        *
        * @param id The identifier to set for the node.
        */
      void SetId(unsigned int id);

      /**
        * Returns the type of the node.
        */
      virtual  IHierarchyNode::Type GetTypeNode() const;


      /**
       * Adds a child node.
       *
       * @param child The child to add.
       */
      virtual void AddChild(IHierarchyNode* child);


      /**
       * Removes a child node.
       *
       * @param child The child to remove.
       */
      virtual void RemoveChild(IHierarchyNode* child);


      /**
       * Removes all child nodes.
       */
      virtual void RemoveAllChildren();



      /**
       * Returns the first child for this node.
       *
       * @return The first child.
       */
      IHierarchyNode* GetFirstChild() const;

      /**
       * Returns the first sibling of this node.
       *
       * @return The first sibling.
       */
      IHierarchyNode* GetNextSibling() const;

      /**
       * Returns the previous sibling to this node.
       *
       * @return The previous sibling.
       */
      IHierarchyNode* GetPreviousSibling() const;

      /**
       * Returns the parent of this node.
       *
       * @return The parent.
       */
      IHierarchyNode* GetParent() const;

      /**
       * Returns the number of direct children of this item.
       *
       * @return The number of children.
       */
      unsigned int GetChildCount() const;


      /**
       * Gets the top level node in this node's parent hierarchy.
       */
      IHierarchyNode* GetRootNode() const;


      /**
       * Gets the boolen node in this node's .
       */
      bool isEnabled() const;




      void ComputeListOldNodes( std::vector<IHierarchyNode*>& _nodes)
      {
          _nodes.push_back(this);
          for (IHierarchyNode* child = _firstChild; child != nullptr; child = _nextSibling)
          {
              child->ComputeListOldNodes(_nodes);
          }
      }

     //=====================================================//


      virtual void Setup();
      virtual void Update(float elapsedTime);


      //=====================================================//


     IHierarchyNode* Clone() const;


     void Copy(const IHierarchyNode *node);



protected:

      /**
     * Constructor.
     */
    IHierarchyNode(unsigned int id);

    /**
     * Destructor.
     */
    virtual ~IHierarchyNode(){}


    /**
     * Removes this node from its parent.
     */
    void remove();



    /**
     * Clones a single node and its data but not its children.
     *
     * @param context The clone context.
     *
     * @return Pointer to the newly created node.
     */
    virtual IHierarchyNode* CloneSingleNode(IHierarchyNodeCloneContext &context) const;

    /**
     * Recursively clones this node and its children.
     *
     * @param context The clone context.
     *
     * @return The newly created node.
     */
    IHierarchyNode* CloneRecursive(IHierarchyNodeCloneContext &context) const;


    /**
     * Copies the data from this node into the given node.
     *
     * @param node The node to copy the data to.
     * @param context The clone context.
     */
    void CloneInto(IHierarchyNode* node, IHierarchyNodeCloneContext &context) const;

};



/**
 * NodeCloneContext represents the context data that is kept when cloning a node.
 * The NodeCloneContext is used to make sure objects don't get cloned twice.
 */
class IHierarchyNodeCloneContext
{
public:

    /**
     * Constructor.
     */
     IHierarchyNodeCloneContext();

    /**
     * Destructor.
     */
    ~IHierarchyNodeCloneContext();

    /**
     * Finds the cloned node of the given node or NULL if this node was not registered with this context.
     *
     * @param node The node to search for the cloned copy of.
     *
     * @return The cloned node or NULL if not found.
     */
    IHierarchyNode* findClonedNode(const IHierarchyNode* node);

    /**
     * Registers the cloned node with this context so that it doens't get cloned twice.
     *
     * @param original The pointer to the original node.
     * @param clone The pointer to the cloned node.
     */
    void registerClonedNode(const IHierarchyNode* original, IHierarchyNode* clone);

private:

    /**
     * Hidden copy constructor.
     */
    IHierarchyNodeCloneContext(const IHierarchyNodeCloneContext&);

    /**
     * Hidden copy assignment operator.
     */
    IHierarchyNodeCloneContext& operator=(const IHierarchyNodeCloneContext&);

    /**
     * @brief _clonedNodes
     */
    std::map<const IHierarchyNode*, IHierarchyNode*> _clonedNodes;
};

}

#endif // IHIERARCHYNODE_H
