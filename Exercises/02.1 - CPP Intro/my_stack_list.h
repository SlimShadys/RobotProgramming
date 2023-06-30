#include <iostream> // we will discover this later
#include <cassert>  // assert.h
using namespace std; // we will discover this later

// nested classes and nested structs are possible
// class: default private
// struct: default public

class MyStackList {
public:
  struct Item {
    Item(int info_=0, Item* next_=0);
    int info;
    Item* next;
  };
  
private:
  Item* _first;
  int _num_elements;
public:  

  MyStackList();

  MyStackList(MyStackList& other);
  ~MyStackList();
  void push(int v);
  void pop();
  void clear();
  void print();
  MyStackList& operator=(const MyStackList& other) ;
  
  inline int  numElements() {return _num_elements;}

protected:
  void _copy(const MyStackList& other);

};

