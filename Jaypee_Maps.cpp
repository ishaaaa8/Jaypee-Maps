#include<bits/stdc++.h>
using namespace std;

//NODE FOR QUEUE IMPLEMETATION
class Node {
public:
    pair<int, int> data;
    Node* next;

    Node(const pair<int, int>& value) : data(value), next(nullptr) {}
};
//QUEUE IMPLEMENTATION
class Queue {
private:
    Node* front;
    Node* rear;

public:
    Queue() : front(nullptr), rear(nullptr) {}

    bool isEmpty() const {
        return front == nullptr;
    }

    void push(const pair<int, int>& value) {
        Node* newNode = new Node(value);

        if (isEmpty()) {
            front = rear = newNode;
        } else {
            rear->next = newNode;
            rear = newNode;
        }
    }

    void pop() {
        if (isEmpty()) {
            cout << "Queue is empty, cannot pop." << endl;
            return;
        }

        Node* temp = front;
        front = front->next;

        if (front == nullptr) {
            rear = nullptr;
        }

        delete temp;
    }

    pair<int, int> frontNode(){
        if (isEmpty()) {
            cout << "Queue is empty, no front element." << endl;
            return make_pair(-1, -1);
        }

        return front->data;
    }

    // Clear the queue by deallocating memory
    void clear() {
        while (!isEmpty()) {
            pop();
        }
    }

    ~Queue() {
        clear();
    }
};

//IMPLEMENTING PRIORITY QUEUE USING BINARY HEAP
class PriorityQueue {
private:
    vector<pair<pair<int, int>, int>> heap;

    int parent(int index) {
        return (index - 1) / 2;
    }

    int leftChild(int index) {
        return 2 * index + 1;
    }

    int rightChild(int index) {
        return 2 * index + 2;
    }

    void swap(int i, int j) {
        pair<pair<int, int>, int> temp = heap[i];
        heap[i] = heap[j];
        heap[j] = temp;
    }

    void heapifyUp(int index) {
        while (index > 0 && heap[parent(index)].first.first > heap[index].first.first) {
            swap(index, parent(index));
            index = parent(index);
        }
        while (index > 0 && heap[parent(index)].first.first == heap[index].first.first &&
               heap[parent(index)].first.second > heap[index].first.second) {
            swap(index, parent(index));
            index = parent(index);
        }
    }

    void heapifyDown(int index) {
        int smallest = index;
        int left = leftChild(index);
        int right = rightChild(index);

        if (left < heap.size() && heap[left].first.first < heap[smallest].first.first) {
            smallest = left;
        } else if (left < heap.size() && heap[left].first.first == heap[smallest].first.first &&
                   heap[left].first.second < heap[smallest].first.second) {
            smallest = left;
        }

        if (right < heap.size() && heap[right].first.first < heap[smallest].first.first) {
            smallest = right;
        } else if (right < heap.size() && heap[right].first.first == heap[smallest].first.first &&
                   heap[right].first.second < heap[smallest].first.second) {
            smallest = right;
        }

        if (smallest != index) {
            swap(index, smallest);
            heapifyDown(smallest);
        }
    }

public:
    PriorityQueue() {}

    bool isEmpty() {
        return heap.empty();
    }

    void push(int distance, int nodeData) {
        pair<int, int> distanceNodePair = make_pair(distance, nodeData);
        pair<pair<int, int>, int> element = make_pair(distanceNodePair, nodeData);
        heap.push_back(element);
        heapifyUp(heap.size() - 1);
    }

    pair<int, int> pop() {
        if (isEmpty()) {
            return make_pair(0, -1); // Return an invalid pair for an empty queue
        }

        pair<int, int> root = heap[0].first;
        heap[0] = heap.back();
        heap.pop_back();
        heapifyDown(0);

        return root;
    }
};
//HEAP FROM FIBONACCI HEAP
class FibonacciHeap {
private:
    struct Node {
        int distance;
        int nodeData;
        Node* parent;
        Node* child;
        Node* left;
        Node* right;
        bool marked;
        int degree;

        Node(int d, int v) : distance(d), nodeData(v), parent(nullptr), child(nullptr),
                             left(this), right(this), marked(false), degree(0) {}
    };

    Node* minNode;
    int size;
    unordered_map<int, Node*> nodeMap;

    Node* mergeHeaps(Node* a, Node* b) {
        if (a == nullptr) return b;
        if (b == nullptr) return a;

        Node* aRight = a->right;
        Node* bLeft = b->left;

        a->right = b;
        b->left = a;

        aRight->left = bLeft;
        bLeft->right = aRight;

        return a->distance < b->distance ? a : b;
    }

    void consolidate() {
        int maxDegree = static_cast<int>(log2(size)) + 1;
        vector<Node*> degreeTable(maxDegree, nullptr);
        Node* nodesToVisit = minNode;

        do {
            Node* current = nodesToVisit;
            nodesToVisit = nodesToVisit->right;

            int degree = current->degree;
            while (degreeTable[degree] != nullptr) {
                Node* other = degreeTable[degree];
                if (current->distance > other->distance) {
                    swap(current, other);
                }
                link(other, current);
                degreeTable[degree] = nullptr;
                degree++;
            }
            degreeTable[degree] = current;
        } while (nodesToVisit != minNode);

        minNode = nullptr;

        for (Node* node : degreeTable) {
            if (node != nullptr) {
                if (minNode == nullptr || node->distance < minNode->distance) {
                    minNode = node;
                }
            }
        }
    }

    void link(Node* child, Node* parent) {
        child->left->right = child->right;
        child->right->left = child->left;

        child->parent = parent;
        if (parent->child == nullptr) {
            parent->child = child;
            child->left = child;
            child->right = child;
        } else {
            Node* temp = parent->child->right;
            parent->child->right = child;
            child->left = parent->child;
            child->right = temp;
            temp->left = child;
        }

        parent->degree++;
        child->marked = false;
    }

public:
    FibonacciHeap() : minNode(nullptr), size(0) {}

    bool isEmpty() {
        return minNode == nullptr;
    }

    void push(int distance, int nodeData) {
        Node* newNode = new Node(distance, nodeData);
        nodeMap[nodeData] = newNode;

        if (minNode == nullptr) {
            minNode = newNode;
        } else {
            minNode = mergeHeaps(minNode, newNode);
        }

        if (newNode->distance < minNode->distance) {
            minNode = newNode;
        }

        size++;
    }

    pair<int, int> pop() {
        if (isEmpty()) {
            return make_pair(0, -1); // Return an invalid pair for an empty heap
        }

        Node* oldMin = minNode;
        int nodeData = oldMin->nodeData;
        if (oldMin->child != nullptr) {
            Node* child = oldMin->child;
            do {
                Node* next = child->right;
                minNode = mergeHeaps(minNode, child);
                child->parent = nullptr;
                child = next;
            } while (child != oldMin->child);
        }

        oldMin->left->right = oldMin->right;
        oldMin->right->left = oldMin->left;

        if (oldMin == oldMin->right) {
            minNode = nullptr;
        } else {
            minNode = oldMin->right;
            consolidate();
        }

        nodeMap.erase(nodeData);
        delete oldMin;
        size--;

        return make_pair(oldMin->distance, nodeData);
    }

    bool decreasePriority(int nodeData, int newDistance) {
        Node* node = nodeMap[nodeData];
        if (node == nullptr || newDistance >= node->distance) {
            return false;
        }

        node->distance = newDistance;
        Node* parent = node->parent;

        if (parent != nullptr && node->distance < parent->distance) {
            cut(node, parent);
            cascadingCut(parent);
        }

        if (node->distance < minNode->distance) {
            minNode = node;
        }

        return true;
    }

    void cut(Node* node, Node* parent) {
        if (node->right == node) {
            parent->child = nullptr;
        } else {
            node->left->right = node->right;
            node->right->left = node->left;
            if (parent->child == node) {
                parent->child = node->right;
            }
        }

        parent->degree--;
        mergeHeaps(minNode, node);
        node->parent = nullptr;
        node->marked = false;
    }

    void cascadingCut(Node* node) {
        Node* parent = node->parent;
        if (parent != nullptr) {
            if (!node->marked) {
                node->marked = true;
            } else {
                cut(node, parent);
                cascadingCut(parent);
            }
        }
    }
};
//SET IMPLEMENTATION-AVL TREE
struct Node1 {
    pair<int, int> key; // Pair of int: {distance, node data}
    int height;
    Node1* left;
    Node1* right;

    Node1(const pair<int, int>& val) : key(val), height(1), left(nullptr), right(nullptr) {}
};

class AVLTree {
private:
    Node1* root;

    int height(Node1* node) {
        return node ? node->height : 0;
    }

    int getBalance(Node1* node) {
        if (node == nullptr) return 0;
        return height(node->left) - height(node->right);
    }

    Node1* rightRotate(Node1* y) {
        Node1* x = y->left;
        Node1* T2 = x->right;

        x->right = y;
        y->left = T2;

        y->height = 1 + max(height(y->left), height(y->right));
        x->height = 1 + max(height(x->left), height(x->right));

        return x;
    }

    Node1* leftRotate(Node1* x) {
        Node1* y = x->right;
        Node1* T2 = y->left;

        y->left = x;
        x->right = T2;

        x->height = 1 + max(height(x->left), height(x->right));
        y->height = 1 + max(height(y->left), height(y->right));

        return y;
    }

    Node1* insertNode(Node1* node, const pair<int, int>& key) {
        if (node == nullptr) {
            return new Node1(key);
        }

        if (key.first < node->key.first) {
            node->left = insertNode(node->left, key);
        } else if (key.first > node->key.first) {
            node->right = insertNode(node->right, key);
        } else {
            return node;
        }

        node->height = 1 + max(height(node->left), height(node->right));
        int balance = getBalance(node);

        if (balance > 1 && key.first < node->left->key.first) {
            return rightRotate(node);
        }

        if (balance > 1 && key.first > node->left->key.first) {
            node->left = leftRotate(node->left);
            return rightRotate(node);
        }

        if (balance < -1 && key.first > node->right->key.first) {
            return leftRotate(node);
        }

        if (balance < -1 && key.first < node->right->key.first) {
            node->right = rightRotate(node->right);
            return leftRotate(node);
        }

        return node;
    }

    Node1* minValueNode(Node1* node) {
        Node1* current = node;

        while (current->left != nullptr) {
            current = current->left;
        }

        return current;
    }

    Node1* erase(Node1* node, const pair<int, int>& key) {
        if (node == nullptr) {
            return node;
        }

        if (key.first < node->key.first) {
            node->left = erase(node->left, key);
        } else if (key.first > node->key.first) {
            node->right = erase(node->right, key);
        } else {
            if (node->left == nullptr || node->right == nullptr) {
                Node1* temp = node->left ? node->left : node->right;

                if (temp == nullptr) {
                    temp = node;
                    node = nullptr;
                } else {
                    *node = *temp;
                }

                delete temp;
            } else {
                Node1* temp = minValueNode(node->right);
                node->key = temp->key;
                node->right = erase(node->right, temp->key);
            }
        }

        if (node == nullptr) {
            return node;
        }

        node->height = 1 + max(height(node->left), height(node->right));
        int balance = getBalance(node);

        if (balance > 1 && getBalance(node->left) >= 0) {
            return rightRotate(node);
        }

        if (balance > 1 && getBalance(node->left) < 0) {
            node->left = leftRotate(node->left);
            return rightRotate(node);
        }

        if (balance < -1 && getBalance(node->right) <= 0) {
            return leftRotate(node);
        }

        if (balance < -1 && getBalance(node->right) > 0) {
            node->right = rightRotate(node->right);
            return leftRotate(node);
        }

        return node;
    }

    bool searchHelper(Node1* node, const pair<int, int>& key) const {
        if (node == nullptr) {
            return false;
        }

        if (key.first < node->key.first) {
            return searchHelper(node->left, key);
        } else if (key.first > node->key.first) {
            return searchHelper(node->right, key);
        } else {
            return true;
        }
    }

    void inorderTraversalHelper(Node1* node) const {
        if (node != nullptr) {
            inorderTraversalHelper(node->left);
            cout << "{" << node->key.first << ", " << node->key.second << "} ";
            inorderTraversalHelper(node->right);
        }
    }

    Node1* topNode(Node1* node) {
        Node1* current = node;
        while (current->right != nullptr) {
            current = current->right;
        }
        return current;
    }

public:
    AVLTree() : root(nullptr) {}

    void insert(const pair<int, int>& key) {
        root = insertNode(root, key);
    }

    void erasee(const pair<int, int>& key) {
        root = erase(root, key);
    }

    bool contains(const pair<int, int>& key) const {
        return searchHelper(root, key);
    }

    void inorderTraversal() const {
        inorderTraversalHelper(root);
        cout << endl;
    }

    pair<int, int> top(){
        if (root == nullptr) {
            throw runtime_error("Tree is empty!");
        }
        Node1* topNodePtr = topNode(root);
        return topNodePtr->key;
    }

    bool isEmpty() const {
        return root == nullptr;
    }

    bool find(const pair<int, int>& key) const {
        return searchHelper(root, key);
    }
};
//GRAPH IMPLEMENTATION
class graph{
public:
    unordered_map<int,list<pair<int,int>>>adjList;
    //node:{(nbr,wt)}
    //ADJACENCY MATRIX FORMATION
    void addEdge(int u,int v,int wt,bool direction,vector<vector<int>>&adjMatrix){

        //adjacencyMatrix[u][v] = wt;
        adjMatrix[u][v] = wt;
        if(direction==0){
            adjMatrix[v][u] = wt;
        }

    }
    //ADJACENCY LIST FORMATION
    void createAdjList(vector<vector<int>>&adjMatrix,int n){
        for(int i=0;i<n;i++){
            for(int j=0;j<n;j++){
                if(adjMatrix[i][j]){
                    adjList[i].push_back({j,adjMatrix[i][j]});
                }
            }
        }
    }
    //PRINTING THE ADJACENCY LIST
    void printAdjList(){
        for(auto node:adjList){
            cout<<node.first<<"->";
            for(auto nbr:node.second){
                cout<<"("<<nbr.first<<","<<nbr.second<<") ";
            }
            cout<<endl;
        }
    }
    //DIJKSTRA USING NORMAL QUEUE
    vector<int> dijkstraQueue(int S,int V)
    {
        //QUEUE OBJECT
        Queue q;
        vector<int> dist(n + 1, 1e9), parent(n + 1);
        for (int i = 1; i <= n; i++)
            parent[i] = i;
        vector<int> distTo(V, INT_MAX);
        distTo[S] = 0;
        q.push({0, S});
        while (!q.isEmpty())
        {
            auto topEle=q.frontNode();
            int node = topEle.second;
            int dis = topEle.first;
            q.pop();

            for (auto it : adjList[node])
            {
                int v = it.first;
                int w = it.second;
                if (dis + w < distTo[v])
                {
                    distTo[v] = dis + w;
                        q.push({dis + w, v});
                         parent[v] = node;
                }
            }
        }
        vector<int> path;
        int node = V-1;

        while (parent[node] != node)
        {
            path.push_back(node);
            node = parent[node];
        }
        path.push_back(1);

        reverse(path.begin(), path.end());
        for(auto it:path) cout<<path<<"->";
        return distTo;
    }
    //DIJKSTRA ALGO USING SET
    vector<int> DijkstraSet(int src,int n){
        vector<int>distance(n,INT_MAX);
        AVLTree st; //pair<dist,node>
        distance[src]=0;
         vector<int> dist(n + 1, 1e9), parent(n + 1);
        for (int i = 1; i <= n; i++)
            parent[i] = i;
        st.insert({0,src});
        while(!st.isEmpty()){
            auto topEle=st.top();
            st.erasee(topEle);
            int topDist=topEle.first;
            int topNode=topEle.second;
            for(auto nbr:adjList[topNode]){
                if(topDist+nbr.second<distance[nbr.first]){
                    auto result=st.find(make_pair(distance[nbr.first],nbr.first));
                    if(result){
                        st.erasee({distance[nbr.first],nbr.first});
                    }
                    distance[nbr.first]=topDist+nbr.second;
                    st.insert(make_pair(distance[nbr.first],nbr.first));
                     parent[nbr.first] = topNode;
                }
            }

        }
        vector<int> path;
        int node = V-1;

        while (parent[node] != node)
        {
            path.push_back(node);
            node = parent[node];
        }
        path.push_back(1);

        reverse(path.begin(), path.end());
        for(auto it:path) cout<<path<<"->";
        return distance;
    }
    //DIJKSTRA USING PRIORITY QUEUE
    vector<int> dijkstraPQ(int S,int V)
    {


        //PRIORITY QUEUE
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

         vector<int> dist(n + 1, 1e9), parent(n + 1);
        for (int i = 1; i <= n; i++)
            parent[i] = i;

        vector<int> distTo(V, INT_MAX);

        distTo[S] = 0;
        pq.push({0, S});

        // Now, pop the minimum distance node first from the min-heap
        // and traverse for all its adjacent nodes.
        while (!pq.empty())
        {
            int node = pq.top().second;
            int dis = pq.top().first;
            pq.pop();


            for (auto it : adjList[node])
            {
                int v = it.first;
                int w = it.second;
                if (dis + w < distTo[v])
                {
                    distTo[v] = dis + w;
                    parent[v]=node;
                    // If current distance is smaller,
                    // push it into the queue.
                    pq.push({dis + w, v});
                }
            }
        }
        vector<int> path;
        int node = V-1;

        while (parent[node] != node)
        {
            path.push_back(node);
            node = parent[node];
        }
        path.push_back(1);

        reverse(path.begin(), path.end());
        for(auto it:path) cout<<path<<"->";
        return distTo;
    }

    //DIJKSTRA USING BINATY HEAP
        vector<int> dijkstraBinaryHeap(int S,int V)
    {


        PriorityQueue pq;
         vector<int> dist(n + 1, 1e9), parent(n + 1);
        for (int i = 1; i <= n; i++)
            parent[i] = i;
        vector<int> distTo(V);
        for(int i=0;i<V;i++){
            distTo[i]=INT_MAX;
        }

        distTo[S] = 0;
        pq.push(0, S);

        while (!pq.isEmpty())
        {
            int node = pq.pop().second;
            int dis = pq.pop().first;

            for (auto it : adjList[node])
            {
                int v = it.first;
                int w = it.second;
                if (dis + w < distTo[v])
                {
                    distTo[v] = dis + w;
                    pq.push(dis + w, v);
                    parent[v]=node;
                }
            }
        }
        vector<int> path;
        int node = V-1;

        while (parent[node] != node)
        {
            path.push_back(node);
            node = parent[node];
        }
        path.push_back(1);

        reverse(path.begin(), path.end());
        for(auto it:path) cout<<path<<"->";

        return distTo;
    }

    //DIJKSTRA USING FIBONACCI HEAP
    vector<int> dijkstraFibonacciHeap(int S,int V)
    {
        FibonacciHeap pq;
         vector<int> dist(n + 1, 1e9), parent(n + 1);
        for (int i = 1; i <= n; i++)
            parent[i] = i;
        vector<int> distTo(V);
        for(int i=0;i<V;i++){
            distTo[i]=INT_MAX;
        }
        distTo[S] = 0;
        pq.push(0, S);

        while (!pq.isEmpty())
        {
            int node = pq.pop().second;
            int dis = pq.pop().first;
            for (auto it : adjList[node])
            {
                int v = it.first;
                int w = it.second;
                if (dis + w < distTo[v])
                {
                    distTo[v] = dis + w;
                    pq.push(dis + w, v);
                    parent[v]=node;
                }
            }
        }
        vector<int> path;
        int node = V-1;

        while (parent[node] != node)
        {
            path.push_back(node);
            node = parent[node];
        }
        path.push_back(1);

        reverse(path.begin(), path.end());
        for(auto it:path) cout<<path<<"->";
        return distTo;
    }


};
void print1()
{
    system("Color 0A");
    cout<<"\n\n\n\t\t\t\t\t       WELCOME TO JAYPEE MAPS";
    cout<<"\n\n\t\t\t\t\t\t EXPLORE OUR CAMPUS";
    cout<<"\n\n\t\t\t\t\t\tFIND YOUR WAY AROUND";
    cout<<"\n\n\t\t\t\t\t\t DISCOVER MORE!!!";
//system("Color 0A");
    cout<<"\n\n\n\n\n\t\t\t\t Press 1 to continue\t\t Press 0 to exit";
    cout<<"\n\n\t\t\t\t\t\tENTER YOUR CHOICE:- ";
}

print2(map<string,int>&mapping,string &src, string &dest)
{
    system("cls");
    system("Color 03");
    cout<<"\n\n\n\n\n\n\n";
    cout<<"\t\t\t\t\t\t\t LOCATION";
    int i=1;
    for(auto it: mapping)
    {
        cout<<"\n\t\t\t\t\t\t\t"<<i<<". "<<it.first;
        i++;
    }
    cout<<"\n\n\t\t\t\t\t\t ENTER YOUR CURRENT LOCATION";
    cout<<"\n\n\t\t\t\t\t\t\t";
    cin>>src;

    cout<<"\n\n\t\t\t\t\t\t ENTER YOUR DESTINATION";
    cout<<"\n\n\t\t\t\t\t\t\t";
    cin>>dest;

}

void print3(int &choice)
{
    system("cls");

    cout<<"\n\n\n\n\t\t\t\t\t\t Tell Us How Do You want To Implement it";
    cout<<"\n\n\t\t\t\t\t\t\t 1. QUEUE";
    cout<<"\n\t\t\t\t\t\t\t 2. SET";
    cout<<"\n\t\t\t\t\t\t\t 3. PRIORITY QUEUE";
    cout<<"\n\t\t\t\t\t\t\t 4. BINARY HEAP";
    cout<<"\n\t\t\t\t\t\t\t 5. FIBONACCI HEAP";

    cout<<"\n\n\n\t\t\t\t\t\t Enter Your Choice:";

    cin>>choice;

}
int main(){

    graph g;
    int n=6;
    vector<vector<int>> adjMatrix(n, vector<int>(n, 0));
    //PUTTING ALL DATA SET INTO ADJ MATRIX
    g.addEdge(0,1,100,0,adjMatrix);
    g.addEdge(1,2,80,0,adjMatrix);
    g.addEdge(2,4,70,0,adjMatrix);
    g.addEdge(2,3,50,0,adjMatrix);
    g.addEdge(1,3,20,0,adjMatrix);
    g.addEdge(0,5,20,0,adjMatrix);
    g.addEdge(5,4,60,0,adjMatrix);
    g.addEdge(3,5,40,0,adjMatrix);
    g.addEdge(0,4,80,0,adjMatrix);
    //PUTTING DATA SET INTO ADJ LIST
    g.createAdjList(adjMatrix,n);


    g.printAdjList();

    //MAPPING FOR VARIOUS LOCATION OF COLLEGE
    map<string,int>mapping;
    mapping.insert({"Gate-1",0});
    mapping.insert({"Gate-2",1});
    mapping.insert({"Gate-3",2});
    mapping.insert({"JBS",4});
    mapping.insert({"ABB-1",3});
    mapping.insert({"Cafe",5});


    print1();
    int ch;
    cin>>ch;
    switch(ch)
    {

    case 1:  break;
    default: exit(0);
    }

    string s,d;

    print2(mapping,s,d);



    while((s!="Gate-1"&&s!="Gate-2"&&s!="Gate-3"&&s!="JBS"&&s!="ABB-1"&&s!="Cafe")&&(d!="Gate-1"&&d!="Gate-2"&&d!="Gate-3"&&d!="JBS"&&d!="ABB-1"&&d!="Cafe"))
    {

        system("cls");
        cout<<"\n\n\n\n\n\n\t\t\t\t\t\t\t INVALID LOCATION";
        cout<<"\n\n\t\t\t\t\t\t Press 1 to renter the choices\n\t\t\t\t\t\t Press 0 to exit";
        cout<<"\n\n\t\t\t\t\t\t\t      ";
        int cho;
        cin>>cho;
        switch(cho)
       {
          case 1:  print2(mapping,src,dest);
                   break;

          default: exit(0);
       }

    }


    int choice;
    print3(choice);

    while(choice!=1 &&choice!=2 &&choice!=3 &&choice!=4 &&choice!=5 )
    {

        system("cls");
        cout<<"\n\n\n\n\n\n\t\t\t\t\t\t INVALID METHOD CHOICE";
        cout<<"\n\n\t\t\t\t\t\t Press 1 to renter the choices\n\t\t\t\t\t\t Press 0 to exit";
        cout<<"\n\n\t\t\t\t\t\t\t      ";
        int cho;
        cin>>cho;
        switch(cho)
       {
          case 1:  print3(choice);
                   break;

          default: exit(0);
       }

    }

    /*cout<<"Enter Source"<<endl;
    string s;
    cin>>s;

    cout<<"Enter Destination"<<endl;
    string d;
    cin>>d;*/
    vector<int>dist;
        switch(choice){
        case 1:
                dist=g.dijkstraQueue(mapping[s],n);
                cout<<"Shortest path using QUEUE :"<<endl;
                for(int i=0;i<n;i++){
                if(i==mapping[d]){
                    cout<<"Shortest Distance to reach "<<d<<" from "<<s<<" is: "<<dist[i];
                }
                cout<<endl;
                }
            break;
        case 2:
            dist=g.DijkstraSet(mapping[s],n);
    cout<<"Shortest path using SET :"<<endl;
    for(int i=0;i<n;i++){
        if(i==mapping[d]){
            cout<<"Shortest Distance to reach "<<d<<" from "<<s<<" is: "<<dist[i];
        }
        cout<<endl;
    }
     break;
        case 3:
            dist=g.dijkstraPQ(mapping[s],n);
    cout<<"Shortest path using Priority queue :"<<endl;
    for(int i=0;i<n;i++){
        if(i==mapping[d]){
            cout<<"Shortest Distance to reach "<<d<<" from "<<s<<" is: "<<dist[i];
        }
        cout<<endl;
    }
    break;
        case 4:
         dist=g.dijkstraBinaryHeap(mapping[s],n);
    cout<<"Shortest path using BINARY HEAP :"<<endl;
    for(int i=0;i<n;i++){
        if(i==mapping[d]){
            cout<<"Shortest Distance to reach "<<d<<" from "<<s<<" is: "<<dist[i];
        }
        cout<<endl;
    }
    break;
        case 5:

            dist=g.dijkstraFibonacciHeap(mapping[s],n);
    cout<<"Shortest path using FIBONACCI HEAP :"<<endl;
    for(int i=0;i<n;i++){
        if(i==mapping[d]){
            cout<<"Shortest Distance to reach "<<d<<" from "<<s<<" is: "<<dist[i];
        }
        cout<<endl;
    }
    break;
        default:
            cout<<"Thanks for visiting"<<endl;
        }

}
