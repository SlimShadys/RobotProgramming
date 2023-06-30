#include <iostream>
#include "my_stack_list.h"

using namespace std; // we will discover this later

// nested classes and nested structs are possible
// class: default private
// struct: default public


MyStackList::Item::Item(int info_, Item* next_):
  info(info_),
  next(next_){}
  
// this stuff accessible only to this class

MyStackList::MyStackList():
  _first(0),
  _num_elements(0) {
  cerr << "MyStackList::ctor [" << this << "]" << endl;
}

MyStackList::MyStackList(MyStackList& other) {
  cerr << "MyStackList::copy ctor [" << this << "]" << endl;
  _copy(other);
}

MyStackList::~MyStackList() {
  cerr << "MyStackList::dtor [" << this << "]" << endl;
  clear();
}

void MyStackList::push(int v) {
  _first=new Item(v, _first); // heap allocation!
  ++_num_elements;
}

void MyStackList::pop() {
  assert(_num_elements);
  Item* deleted=_first;
  _first=_first->next;
  --_num_elements;
  delete deleted; // < this clears an object and calls the destructor chain
}

void MyStackList::clear() {
  while (numElements())
    pop();
}


void MyStackList::print(){
  cerr << "MyStackList::print() [" << this << "]" << endl;
  Item* aux=_first;
  int k=0;
  while(aux) {
    cerr << "[" <<k <<"]: " << aux->info << endl;
    ++k;
    aux=aux->next;
  }
}
  

  
  // this en0ables us to assign stack lists!
MyStackList& MyStackList::operator=(const MyStackList& other) {
  cerr << "MyStackList::operator = [" << this << "]" << endl;
  if (&other==this)
    return *this; 
  clear();
  _copy(other);
  return *this; // this enables assigment expressions
}

void MyStackList::_copy(const MyStackList& other) {
  _first=0;
  _num_elements=0;
  const Item* aux=other._first;
  Item** last(&_first); // reference to pointer
  // deep copy
  while (aux) {
    *last=new Item(aux->info);
    last=&((*last)->next);
    aux=aux->next;
  }
  _num_elements=other._num_elements;
}


