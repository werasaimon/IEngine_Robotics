#ifndef IHierarchySceneWORLD_H
#define IHierarchySceneWORLD_H

#include "IHierarchyNode.h"
#include <string>

namespace IEngine
{


class IHierarchyScene
{
    public:

        /**
         * Creates a new empty scene.
         *
         * @param id ID of the new scene, or NULL to use an empty string for the ID (default).
         *
         * @return The newly created empty scene.
         * @script{create}
         */
        static IHierarchyScene* Create(const char* id = NULL);


        /**
         * Gets a currently active scene.
         *
         * If id is an NULL, the first active scene is returned.
         *
         * @param id ID of the scene to retrieve, or NULL to retrieve the first active scene.
         *
         * @return The scene that matches the specified ID, or NULL if no matching scene could be found.
         */
        static IHierarchyScene* GetScene(const char* id = NULL);

        /**
         * Gets the identifier for the scene.
         *
         * @return The scene identifier.
         */
        const char* GetId() const;

        /**
         * Sets the identifier for the scene.
         *
         * @param id The identifier to set for the scene.
         */
        void SetId(const char* id);



        /**
         * Creates and adds a new node to the scene.
         *
         * @param id An optional node ID.
         *
         * @return The new node.
         */
        IHierarchyNode* AddNode(unsigned int id = 0);

        /**
         * Adds the specified node to the scene.
         *
         * @param node The node to be added to the scene.
         */
        void AddNode(IHierarchyNode* node);

        /**
         * Removes the specified node from the scene.
         *
         * @param node The node to remove.
         */
        void RemoveNode(IHierarchyNode* node);

        /**
         * Removes all nodes from the scene.
         */
        void RemoveAllNodes();

        /**
         * Returns the number of nodes at the root level of the scene.
         *
         * @return The node count.
         */
        unsigned int GetNodeCount() const;

        /**
         * Returns the first node in the scene.
         *
         * @return The first node in the scene.
         */
        IHierarchyNode* GetFirstNode() const;



        std::vector<IHierarchyNode*> ListOldNodes()
        {
            std::vector<IHierarchyNode*> List;
            for (IHierarchyNode* node = _firstNode; node != NULL; node = node->_nextSibling)
            {
               node->ComputeListOldNodes(List);
            }
            return List;
        }



        /**
         * Updates all active nodes in the scene.
         *
         * This method is recursively calls the Node::update(float) method on all nodes that
         * are active within the scene. A Node is considered active if Node::isActive()
         * returns true.
         *
         * @param elapsedTime Elapsed time in milliseconds.
         */
        void Update(float elapsedTime);



        /**
         * @see VisibleSet#reset
         */
        void Reset();



     protected:

        /**
         * Constructor.
         */
        IHierarchyScene();

        /**
         * Hidden copy constructor.
         */
        IHierarchyScene(const IHierarchyScene& copy);

        /**
         * Destructor.
         */
        virtual ~IHierarchyScene();

        /**
         * Hidden copy assignment operator.
         */
        IHierarchyScene& operator=(const IHierarchyScene&);

    private:


        std::string _id;

        IHierarchyNode* _firstNode;
        IHierarchyNode* _lastNode;
        unsigned int _nodeCount;
       // bool _bindAudioListenerToCamera;
        IHierarchyNode* _nextItr;
        bool _nextReset;
};


}



#endif // IHierarchySceneWORLD_H
