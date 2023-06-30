#include <iostream> // we will discover this later
#include <cassert>  // assert.h
using namespace std; // we will discover this later

// nested classes and nested structs are possible
// class: default private
// struct: default public

class MyStackList {
  // public part, all below this label is accessible outside
public:
  // struct declared inside class
  struct Item {
    // default args in function, override default ctor
    Item(int info_=0, Item* next_=0):
      info(info_),
      next(next_){}
    int info;
    Item* next;
  };
  
// this stuff accessible only to this class
private:
  Item* _first;
  int _num_elements;

  // sections might interleave
public:  

  MyStackList():
    _first(0),
    _num_elements(0) {
    cerr << "MyStackList::ctor [" << this << "]" << endl;
  }

  MyStackList(MyStackList& other) {
    cerr << "MyStackList::copy ctor [" << this << "]" << endl;
    _copy(other);
  }

  ~MyStackList() {
    cerr << "MyStackList::dtor [" << this << "]" << endl;
    clear();
  }

  void push(int v) {
    _first=new Item(v, _first); // heap allocation!
    ++_num_elements;
  }

  void pop() {
    assert(_num_elements);
    Item* deleted=_first;
    _first=_first->next;
    --_num_elements;
    delete deleted; // < this clears an object and calls the destructor chain
  }
  
  int  numElements() {return _num_elements;}

  void clear() {
    while (numElements())
      pop();
  }


  void print(){
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
  MyStackList& operator=(const MyStackList& other) {
    cerr << "MyStackList::operator = [" << this << "]" << endl;
    if (&other==this)
      return *this; 
    clear();
    _copy(other);
    return *this; // this enables assigment expressions
  }

  // protected part, all below this label is accessible only to derived classes
protected:
  void _copy(const MyStackList& other) {
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


};


int main(int argc, const char** argv) {
  MyStackList sl1;
  sl1.push(11);
  sl1.push(22);
  sl1.push(33);
  sl1.push(44);
  sl1.print();

  // copy ctor
  MyStackList sl2=sl1;
  sl2.push(55);
  sl2.print();

  sl2=sl1;
  sl2.print();
}
