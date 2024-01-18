#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <ostream>
#include <string>
#include <cstdlib>
#include <exception>

//Named CONSTANST

const std::string NODE = "node";
const std::string EDGE = "edge";

//EXCEPTION CLASS 
class Exception {
private:
	std::string message_;
public:
	Exception(const std::string& message);
	const char* message() const;
};

Exception::Exception(const std::string& message) : message_(message) {}
const char* Exception::message() const {
	return &(message_[0]); // Correct way is ( message_.c_str() )
}

class MemoryException final : public Exception {
public:
	using Exception::Exception;
};

class IdentifierException final : public Exception {
public:
	using Exception::Exception;
};

class ElementException final : public Exception {
public:
	using Exception::Exception;
};

class ConflictException final : public Exception {
public:
	using Exception::Exception;
};

class FileException final : public Exception {
public:
	using Exception::Exception;
};

//GRAPHS
using Identifier = size_t;

enum class GraphType {
	UNDIRECTED,
	DIRECTED
};

//ADDING CLASSES AND DECLARAING THEIR FUNCTIONS
template<typename NData>
class Node {
private:
	Identifier id_;
	NData data_;
public:

	Node(Identifier id, const NData& data);
	Node(Identifier id, NData&& data);


	~Node() = default;
	Identifier getId() const;
	NData& getData();
	const NData& getData() const;
};

template<typename EData>
class Edge {
private:
	Identifier id_;
	Identifier sourceNode_;
	Identifier targetNode_;
	EData data_;

public:
	Edge(Identifier id, Identifier sourceNode, Identifier targetNode, EData data);
	~Edge() = default;
	Identifier getId() const;
	Identifier getSource() const;
	Identifier getTarget() const;
	EData& getData();
	const EData& getData() const;
};

template<typename NData, typename EData>
class Graph;


template<typename NData, typename EData>
class Nodes{

public:

	Graph<NData, EData>* graph_;
	std::vector<Node<NData>> nodesList_;

	Nodes(Graph<NData, EData>* graph) : graph_(graph) {};


	const std::vector<Node<NData>>& getNodesList() const{
		return nodesList_;
	}

	size_t size() const;
	bool exists(Identifier id) const;

	Node<NData>& get(Identifier id);
	const Node<NData>& get(Identifier id) const;

	Node<NData>& operator[](size_t id);
	const Node<NData>& operator[](size_t id) const;

	void print(std::ostream& stream = std::cout) const;

	Node<NData>& add(Identifier id, const NData& data);
	Node<NData>& add(Identifier id, NData&& data);
	Node<NData>& add(const NData& data);
	Node<NData>& add(NData&& data);

	typename std::vector<Node<NData>>::iterator begin();
	typename std::vector<Node<NData>>::iterator end();
	typename std::vector<Node<NData>>::const_iterator begin() const;
	typename std::vector<Node<NData>>::const_iterator end() const;
};


template<typename NData, typename EData>
class Edges{
protected:
	class Request;

public:
	Graph<NData, EData>* graph_;

	std::vector<Edge<EData>> edgeList_;
	std::vector<std::vector<Identifier>> adjacencyMatrix_;

	Edges(Graph<NData, EData>* graph) : graph_(graph) {};

	const std::vector<Edge<EData>>& getEdgeList() const {
		return edgeList_;
	}

	size_t size() const;

	bool exists(Identifier id) const;
	bool exists(Identifier source, Identifier target) const;

	Edge<EData>& get(Identifier id);
	const Edge<EData>& get(Identifier id) const;

	Edge<EData>& get(Identifier source, Identifier target);
	const Edge<EData>& get(Identifier source, Identifier target) const;

	void printMatrix(std::ostream& stream = std::cout) const;

	const typename Edges<NData, EData>::Request operator[](size_t row) const;
	typename Edges<NData, EData>::Request operator[](size_t row);

	void print(std::ostream& stream = std::cout) const;

	Edge<EData>& add(Identifier id, Identifier source, Identifier target, const EData& data);
	Edge<EData>& add(Identifier id, Identifier source, Identifier target, EData&& data);
	Edge<EData>& add(Identifier source, Identifier target, const EData& data);
	Edge<EData>& add(Identifier source, Identifier target, EData&& data);

	typename std::vector<Edge<EData>>::iterator begin();
	typename std::vector<Edge<EData>>::iterator end();
	typename std::vector<Edge<EData>>::const_iterator begin() const;
	typename std::vector<Edge<EData>>::const_iterator end() const;
};

template<typename NData, typename EData>
class Edges<NData, EData>::Request {
private:
	friend Edges;
	const Edges<NData, EData>& edges_;
	
	size_t row_;

	Request(const Edges<NData, EData>& edges, size_t row);

public:

	const Edge<EData>& operator[](size_t column) const;
	Edge<EData>& operator[](size_t column);

};



template<typename NData, typename EData>
class Graph {
protected:
	Nodes<NData, EData> nodes_ = Nodes<NData, EData>(this);
	Edges<NData, EData> edges_ = Edges<NData, EData>(this);

	friend Nodes<NData, EData>;
	friend Edges<NData, EData>;

public:
	std::vector<std::vector<Identifier>> adjacencyMatrix_;

	virtual ~Graph() = default;

	Graph();

	Graph(const Graph<NData, EData>& other);
	Graph(Graph<NData, EData>&& other) noexcept;

	Graph<NData, EData>& operator=(const Graph<NData, EData>& other);
	Graph<NData, EData>& operator=(Graph<NData, EData>&& other) noexcept;

	Nodes<NData, EData>& nodes();
	Edges<NData, EData>& edges();
	const Nodes<NData, EData>& nodes() const;
	const Edges<NData, EData>& edges() const;

	void extendAdjacencyMatrix();
	void updateAdjacencyMatrix();

	void print(std::ostream& stream = std::cout) const;
	void print(const std::string& filename) const;

	void import(std::istream& stream = std::cin);
	void import(const std::string& filename);

	void importNode(std::string& sentence);
	void importEdge(std::string& sentence);

	virtual GraphType getDirection() const = 0;
};

template<typename NData, typename EData>
class UndirectedGraph final : public Graph<NData, EData> {
public:
	GraphType getDirection() const override;
};


template<typename NData, typename EData>
class DirectedGraph final : public Graph<NData, EData> {
public:
	GraphType getDirection() const override;
};


//	WE START DEFINING FUNCTIONS FOR ALL THE CLASSES


//Defining NODE class functions
template<typename NData>
Node<NData>::Node(Identifier id, const NData& data) : id_(id), data_(data) {}

template<typename NData>
Node<NData>::Node(Identifier id, NData&& data) : id_(id), data_(data) {}

template<typename NData>
Identifier Node<NData>::getId() const {
	return id_;
}

template<typename NData>
NData& Node<NData>::getData() {
	return data_;
}

template<typename NData>
const NData& Node<NData>::getData() const {
	return data_;
}

template<typename NData>
std::ostream& operator<<(std::ostream& stream, const Node<NData>& node) {
	stream << "node (" << node.getId() << " {" << node.getData() << "})";
	return stream;
}

//Defining EDGE class functions
template<typename EData>
Edge<EData>::Edge(Identifier id, Identifier sourceNode, Identifier targetNode, EData data) : id_(id), sourceNode_(sourceNode), targetNode_(targetNode), data_(data) {}

template<typename EData>
Identifier Edge<EData>::getId() const {
	return id_;
}

template<typename EData>
Identifier Edge<EData>::getSource() const {
	return sourceNode_;
}

template<typename EData>
Identifier Edge<EData>::getTarget() const {
	return targetNode_;
}

template<typename EData>
EData& Edge<EData>::getData() {
	return data_;
}

template<typename EData>
const EData& Edge<EData>::getData() const {
	return data_;
}

template<typename EData>
std::ostream& operator<<(std::ostream& stream, const Edge<EData>& edge) {
	stream << "edge (" << edge.getSource() << ")-[" << edge.getId() << " {" << edge.getData() << "}]->(" << edge.getTarget() << ")";
	return stream;
}

//Defining NODES class functions

template<typename NData, typename EData>
void Nodes<NData, EData>::print(std::ostream& stream) const {
	for (const auto& node : nodesList_) {
		stream << node << std::endl;
	}
}

template<typename NData, typename EData>
std::ostream& operator<<(std::ostream& stream, const Nodes<NData, EData>& nodes) {
	nodes.print(stream);
	return stream;
}
template<typename NData, typename EData>
Node<NData>& Nodes<NData, EData>::add(Identifier id, const NData& data) {
	if (nodesList_.size() < id) {
		throw IdentifierException("Invalid node identifier "+ std::to_string(id) +" requested");
	}

	if (exists(id)) {
		throw ConflictException("Node with identifier " + std::to_string(id) + " already exists");
	}

	try {
		nodesList_.push_back(Node<NData>(id, data));
	}
	catch (const std::bad_alloc&) {
		throw MemoryException("Unavailable memory for a new node in the nodes container");
	}

	graph_->extendAdjacencyMatrix();

	return nodesList_.back();
}

template<typename NData, typename EData>
Node<NData>& Nodes<NData, EData>::add(Identifier id, NData&& data) {
	if (nodesList_.size() < id) {
		throw IdentifierException("Invalid node identifier " + std::to_string(id) + " requested");
	}

	if (exists(id)) {
		throw ConflictException("Node with identifier " + std::to_string(id) + " already exists");
	}

	try {
		nodesList_.push_back(Node<NData>(id, data));	
	}
	catch (const std::bad_alloc&) {
		throw MemoryException("Unavailable memory for a new node in the nodes container");
	}

	graph_->extendAdjacencyMatrix();

	return nodesList_.back();
}

template<typename NData, typename EData>
Node<NData>& Nodes<NData, EData>::add(const NData& data) {
	if (nodesList_.empty()) {
		try {
			nodesList_.push_back(Node<NData>(0, data));
		}
		catch (const std::bad_alloc&) {
			throw MemoryException("Unavailable memory for a new node in the nodes container");
		}
	}
	else {
		Node<NData> tempNode = nodesList_.back();
		try {
			nodesList_.push_back(Node<NData>(tempNode.getId() + 1, data));
		}
		catch (const std::bad_alloc& ) {
			throw MemoryException("Unavailable memory for a new node in the nodes container");
		}
	}

	graph_->extendAdjacencyMatrix();

	return nodesList_.back();
}

template<typename NData, typename EData>
Node<NData>& Nodes<NData, EData>::add(NData&& data) {

	if (nodesList_.empty()) {
		try {
			nodesList_.push_back(Node<NData>(0, data));
		}
		catch (const std::bad_alloc& ) {
			throw MemoryException("Unavailable memory for a new node in the nodes container");
		}
	}
	else {
		Node<NData> tempNode = nodesList_.back();
		try {

			nodesList_.push_back(Node<NData>(tempNode.getId() + 1, data));
		}
		catch (const std::bad_alloc& ) {
			throw MemoryException("Unavailable memory for a new node in the nodes container");
		}
	}

	graph_->extendAdjacencyMatrix();

	return nodesList_.back();
}

template<typename NData, typename EData>
size_t Nodes<NData, EData>::size() const {
	return nodesList_.size();
}

template<typename NData, typename EData>
bool Nodes<NData, EData>::exists(Identifier id) const {
	if (id < size()) {
		return true;
	}

	return false;
}

template<typename NData, typename EData>
Node<NData>& Nodes<NData, EData>::get(Identifier id) {
	if (!exists(id)) {
		throw ElementException("Node with identifier "+ std::to_string(id) +" does not exist");
	}
	return nodesList_[id];
}

template<typename NData, typename EData>
const Node<NData>& Nodes<NData, EData>::get(Identifier id) const {
	if (!exists(id)) {
		throw ElementException("Node with identifier " + std::to_string(id) + " does not exist");
	}
	return nodesList_[id];
}

template<typename NData, typename EData>
Node<NData>& Nodes<NData, EData>::operator[](size_t id){
	if (!exists(id)) {
		throw ElementException("Node with identifier " + std::to_string(id) + " does not exist");
	}
	return nodesList_[id];
}

template<typename NData, typename EData>
const Node<NData>& Nodes<NData, EData>::operator[](size_t id) const {
	if (!exists(id)) {
		throw ElementException("Node with identifier " + std::to_string(id) + " does not exist");
	}
	return nodesList_[id];
}

template<typename NData, typename EData>
typename std::vector<Node<NData>>::iterator Nodes<NData, EData>::begin() {
	return nodesList_.begin();
}

template<typename NData, typename EData>
typename std::vector<Node<NData>>::iterator Nodes<NData, EData>::end() {
	return nodesList_.end();
}

template<typename NData, typename EData>
typename std::vector<Node<NData>>::const_iterator Nodes<NData, EData>::begin() const {
	return nodesList_.begin();
}

template<typename NData, typename EData>
typename std::vector<Node<NData>>::const_iterator Nodes<NData, EData>::end() const {
	return nodesList_.end();
}


//Defining EDGES class functions

template<typename NData, typename EData>
void Edges<NData, EData>::print(std::ostream& stream) const {
	for (const auto& edge : edgeList_) {
		stream << edge << std::endl;
	}
}

template<typename NData, typename EData>
std::ostream& operator<<(std::ostream& stream, const Edges<NData, EData>& edges) {
	edges.print(stream);
	return stream;
}

template<typename NData, typename EData>
void Edges<NData, EData>::printMatrix(std::ostream& stream) const {
	size_t size = graph_->adjacencyMatrix_.size();

	for (size_t i = 0; i < size; i++) {
		for (size_t j = 0; j < size; j++) {

			if (graph_->adjacencyMatrix_[i][j] == 0) {
				stream << "-";
			}
			else {
				stream << graph_->adjacencyMatrix_[i][j] - 1;
			}

			if (j != size - 1) {
				stream << "|";
			}
		}

		stream << std::endl;
	}
}

template<typename NData, typename EData>
Edge<EData>& Edges<NData, EData>::add(Identifier id, Identifier source, Identifier target, const EData& data) {
	size_t sizeAdj = graph_->adjacencyMatrix_.size();

	if (id > edgeList_.size()) {
		throw IdentifierException("Invalid edge identifier "+std::to_string(id) + " requested");
	}

	if (exists(id)) {
		throw ConflictException("Edge with identifier " + std::to_string(id) + " already exists");
	}

	if (source >= sizeAdj) {
		throw ElementException("Source node with identifier " + std::to_string(source) + " does not exist");
	}

	if (target >= sizeAdj) {
		throw ElementException("Target node with identifier " + std::to_string(target) + " does not exist");
	}

	if (exists(source, target)) {
		throw ConflictException("Edge between nodes " + std::to_string(source) + " and " + std::to_string(target) + " already exists");
	}

	try {
		edgeList_.push_back(Edge<EData>(id, source, target, data));
	}
	catch (const std::bad_alloc&) {
		throw MemoryException("Unavailable memory for a new edge in the edges container");
	}
	graph_->updateAdjacencyMatrix();
	return edgeList_.back();
}

template<typename NData, typename EData>
Edge<EData>& Edges<NData, EData>::add(Identifier id, Identifier source, Identifier target, EData&& data) {
	add(id, source, target, data);
}

template<typename NData, typename EData>
Edge<EData>& Edges<NData, EData>::add(Identifier source, Identifier target, const EData& data) {
	size_t sizeAdj = graph_->adjacencyMatrix_.size();

	if (source >= sizeAdj) {
		throw ElementException("Source node with identifier " + std::to_string(source) + " does not exist");
	}

	if (target >= sizeAdj) {
		throw ElementException("Target node with identifier " + std::to_string(target) + " does not exist");
	}

	if (exists(source, target)) {
		throw ConflictException("Edge between nodes " + std::to_string(source) + " and " + std::to_string(target) + " already exists");
	}


	if (edgeList_.empty()) {
		try{
			edgeList_.push_back(Edge<EData>(0, source, target, data));
		}
		catch (const std::bad_alloc&) {
			throw MemoryException("Unavailable memory for a new edge in the edges container");
		}
	}
	else {
		Edge<EData> tempEdge = edgeList_.back();
		try{
			edgeList_.push_back(Edge<EData>(tempEdge.getId() + 1, source, target, data));
		}
		catch (const std::bad_alloc&) {
			throw MemoryException("Unavailable memory for a new edge in the edges container");
		}
	}
	graph_->updateAdjacencyMatrix();
	return edgeList_.back();
}

template<typename NData, typename EData>
Edge<EData>& Edges<NData, EData>::add(Identifier source, Identifier target, EData&& data) {
	add(source, target, data);
}

template<typename NData, typename EData>
size_t Edges<NData, EData>::size() const {
	return edgeList_.size();
}

template<typename NData, typename EData>
bool Edges<NData, EData>::exists(Identifier id) const {
	if (id < size()) {
		return true;
	}
	return false;
}

template<typename NData, typename EData>
bool Edges<NData, EData>::exists(Identifier source, Identifier target) const {
	size_t maxNodeId = graph_->adjacencyMatrix_.size();

	if (source >= maxNodeId) {
		throw ElementException("Source node with identifier " + std::to_string(source) + " does not exist");
	}

	if (target >= maxNodeId) {
		throw ElementException("Target node with identifier " + std::to_string(target) + " does not exist");
	}

	if (graph_->adjacencyMatrix_[source][target] > 0) {
		return true;
	}
	return false;
}

template<typename NData, typename EData>
Edge<EData>& Edges<NData, EData>::get(Identifier id) {
	if (!exists(id)) {
		throw ElementException("Edge with identifier " + std::to_string(id) + " does not exist");
	}
	return edgeList_[id];
}

template<typename NData, typename EData>
const Edge<EData>& Edges<NData, EData>::get(Identifier id) const {
	if (!exists(id)) {
		throw ElementException("Edge with identifier " + std::to_string(id) + " does not exist");
	}
	return edgeList_[id];
}

template<typename NData, typename EData>
Edge<EData>& Edges<NData, EData>::get(Identifier source, Identifier target) {
	size_t maxNodeId = graph_->adjacencyMatrix_.size();
	if (source >= maxNodeId) {
		throw ElementException("Source node with identifier "+ std::to_string(source) + " does not exist");
	}

	if (target >= maxNodeId) {
		throw ElementException("Target node with identifier " + std::to_string(target) + " does not exist");
	}

	if (graph_->adjacencyMatrix_[source][target] == 0) {
		throw ElementException("Edge between nodes " + std::to_string(source) + " and " + std::to_string(target) + " does not exist");
	}

	Identifier id = graph_->adjacencyMatrix_[source][target] - 1;
	return edgeList_[id];
}

template<typename NData, typename EData>
const Edge<EData>& Edges<NData, EData>::get(Identifier source, Identifier target) const {
	size_t maxNodeId = graph_->adjacencyMatrix_.size();
	if (source >= maxNodeId) {
		throw ElementException("Source node with identifier " + std::to_string(source) + " does not exist");
	}

	if (target >= maxNodeId) {
		throw ElementException("Target node with identifier " + std::to_string(target) + " does not exist");
	}

	if (graph_->adjacencyMatrix_[source][target] == 0) {
		throw ElementException("Edge between nodes " + std::to_string(source) + " and " + std::to_string(target) + " does not exist");
	}

	Identifier id = graph_->adjacencyMatrix_[source][target] - 1;
	return edgeList_[id];
}

template<typename NData, typename EData>
typename std::vector<Edge<EData>>::iterator Edges<NData, EData>::begin() {
	return edgeList_.begin();
}

template<typename NData, typename EData>
typename std::vector<Edge<EData>>::iterator Edges<NData, EData>::end() {
	return edgeList_.end();
}

template<typename NData, typename EData>
typename std::vector<Edge<EData>>::const_iterator Edges<NData, EData>::begin() const {
	return edgeList_.begin();
}

template<typename NData, typename EData>
typename std::vector<Edge<EData>>::const_iterator Edges<NData, EData>::end() const {
	return edgeList_.end();
}

//Defining GRAPH class functions

template<typename NData, typename EData>
Nodes<NData, EData>& Graph<NData, EData>::nodes() {
	return nodes_;
}

template<typename NData, typename EData>
Edges<NData, EData>& Graph<NData, EData>::edges() {
	return edges_;
}

template<typename NData, typename EData>
const Nodes<NData, EData>& Graph<NData, EData>::nodes() const {
	return nodes_;
}

template<typename NData, typename EData>
const Edges<NData, EData>& Graph<NData, EData>::edges() const {
	return edges_;
}



template<typename NData, typename EData>
std::ostream& operator<<(std::ostream& stream, const Graph<NData, EData>& graph) {
	graph.print(stream);
	return stream;
}

template<typename NData, typename EData>
void Graph<NData, EData>::print(const std::string& filename) const {
	std::ofstream file;

	file.open(filename);

	if (!file.good()) {
		throw FileException("Unable to open output file "+filename);
	}

	print(file);
	file.close();
}

template<typename NData, typename EData>
void Graph<NData, EData>::print(std::ostream& stream) const {
	nodes_.print(stream);
	edges_.print(stream);
}

template<typename NData, typename EData>
void Graph<NData, EData>::extendAdjacencyMatrix() {
	try {
		for (auto& vec : adjacencyMatrix_) {
			vec.push_back(0);
		}

		adjacencyMatrix_.push_back(std::vector<Identifier>(nodes_.size(), 0));
	}
	catch (const std::bad_alloc&) {
		throw MemoryException("Unavailable memory for the adjacency matrix extension");
	}
}

template<typename NData, typename EData>
void Graph<NData, EData>::updateAdjacencyMatrix() {
	Edge<EData> tempEdge = edges_.getEdgeList().back();
	adjacencyMatrix_[tempEdge.getSource()][tempEdge.getTarget()] = tempEdge.getId() + 1;

	if (getDirection() == GraphType::UNDIRECTED) {
		adjacencyMatrix_[tempEdge.getTarget()][tempEdge.getSource()] = tempEdge.getId() + 1;
	}
}


template<typename NData, typename EData>
void Graph<NData, EData>::import(std::istream& stream ) {
	std::string line;

	while (std::getline(stream, line)) {
		if (!line.empty()) {
			std::istringstream lineStream(line);
			std::string type;
			lineStream >> type;
			if (type == NODE) {
				importNode(line);
			}
			else if (type == EDGE) {
				importEdge(line);
			}
		}
	}
}

template<typename NData, typename EData>
void Graph<NData, EData>::import(const std::string& filename) {
	std::ifstream file;

	file.open(filename);

	if (!file.good()) {
		throw FileException("Unable to open input file "+filename);
	}

	import(file);
	file.close();
}

template<typename NData, typename EData>
void Graph<NData, EData>::importNode(std::string& sentence) {
	Identifier id;
	NData data;

	std::string idStr, dataStr;

	size_t idStart = sentence.find("(") + 1;
	size_t idEnd = sentence.find(" {");

	size_t dataStart = sentence.find("{") + 1;
	size_t dataEnd = sentence.find("}");

	idStr = sentence.substr(idStart, idEnd - idStart);
	dataStr = sentence.substr(dataStart, dataEnd - dataStart);

	std::istringstream toConvertId(idStr);
	toConvertId >> id;

	std::istringstream toConvertData(dataStr);
	toConvertData >> data;

	nodes_.add(id, data);
}


template<typename NData, typename EData>
void Graph<NData, EData>::importEdge(std::string& sentence) {
	Identifier id, source, target;
	EData data;

	std::string idStr, sourceStr, targetStr, dataStr;

	size_t sourceStart = sentence.find("(") + 1;
	size_t sourceEnd = sentence.find(")-");

	size_t idStart = sentence.find("[") + 1;
	size_t idEnd = sentence.find(" {");

	size_t dataStart = sentence.find("{") + 1 ;
	size_t dataEnd = sentence.find("}") ;

	size_t targetStart = sentence.find("]->(") + 4;
	size_t targetEnd = sentence.find(")");

	sourceStr = sentence.substr(sourceStart, sourceEnd - sourceStart);
	idStr = sentence.substr(idStart, idEnd - idStart);
	dataStr = sentence.substr(dataStart, dataEnd - dataStart);
	targetStr = sentence.substr(targetStart, targetEnd - targetStart);

	std::istringstream toConvertId(idStr);
	toConvertId >> id;

	std::istringstream toConvertSource(sourceStr);
	toConvertSource >> source;

	std::istringstream toConvertTarget(targetStr);
	toConvertTarget >> target;

	std::istringstream toConvertData(dataStr);
	toConvertData >> data;

	edges_.add(id, source, target, data);
}


template<typename NData, typename EData>
Graph<NData, EData>::Graph(){}

template<typename NData, typename EData>
Graph<NData, EData>::Graph(const Graph<NData, EData>& other) {
	nodes_ = other.nodes_;
	edges_ = other.edges_;
	nodes_.graph_ = this;
	edges_.graph_ = this;
	adjacencyMatrix_ = other.adjacencyMatrix_;

}

template<typename NData, typename EData>
Graph<NData, EData>::Graph(Graph<NData, EData>&& other) noexcept {
	nodes_.nodesList_ = std::move(other.nodes().nodesList_);
	edges_.edgeList_ = std::move(other.edges().edgeList_);
	nodes_.graph_ = this;
	edges_.graph_ = this;
	adjacencyMatrix_ = std::move(other.adjacencyMatrix_);
}

template<typename NData, typename EData>
Graph<NData, EData>& Graph<NData, EData>::operator=(const Graph<NData, EData>& other) {
	nodes_ = other.nodes_;
	edges_ = other.edges_;
	nodes_.graph_ = this;
	edges_.graph_ = this;
	adjacencyMatrix_ = other.adjacencyMatrix_;

	return *this;
}

template<typename NData, typename EData>
Graph<NData, EData>& Graph<NData, EData>::operator=(Graph<NData, EData>&& other) noexcept {

	nodes_.nodesList_ = std::move(other.nodes_.nodesList_);
	edges_.edgeList_ = std::move(other.edges_.edgeList_);
	adjacencyMatrix_ = std::move(other.adjacencyMatrix_);

	nodes_.graph_ = this;
	edges_.graph_ = this;

	return *this;
}

//Defining UNDIRECTEDGRAPH class functions

template<typename NData, typename EData>
GraphType UndirectedGraph<NData, EData>::getDirection() const { return GraphType::UNDIRECTED; };

//Defining DIRECTEDGRAPH class functions

template<typename NData, typename EData>
GraphType DirectedGraph<NData, EData>::getDirection() const { return GraphType::DIRECTED; }


//Defining functions for double indexing

template<typename NData, typename EData>
Edges<NData, EData>::Request::Request(const Edges<NData, EData>& edges, size_t row) : edges_(edges), row_(row) {};

template<typename NData, typename EData>
const Edge<EData>& Edges<NData, EData>::Request::operator[](size_t column) const {
	return edges_.get(row_, column);
}

template<typename NData, typename EData>
Edge<EData>& Edges<NData, EData>::Request::operator[](size_t column) {
	return const_cast<Edge<EData>&>(edges_.get(row_, column));
}

template<typename NData, typename EData>
const typename Edges<NData, EData>::Request Edges<NData, EData>::operator[](size_t row) const {
	return Request(*this, row);
}
template<typename NData, typename EData>
typename Edges<NData, EData>::Request Edges<NData, EData>::operator[](size_t row) {
	return Request(*this, row);
}

#endif // !GRAPH_H