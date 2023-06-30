#include <iostream> // we will discover this later
#include <cassert>  // assert.h
#include <string>   // string from stl
using namespace std; // we will discover this later

//Types can be parameters!

// template < arguments>

// arguments:
//   - valuestypes (int, float, new defined classes)
//   - type arguments : typename, class, struct
// /// stuff to be templatized
// /// either class or function


template <typename ValueType_>
class MyStackList_ {
  // public part, all below this label is accessible outside
public:
  using ValueType = ValueType_;
  
  // struct declared inside class
  struct Item {
    // default args in function, override default ctor
    Item(ValueType info_=0, Item* next_=0):
      info(info_),
      next(next_){}
    ValueType info;
    Item* next;
  };
  
// this stuff accessible only to this class
private:
  Item* _first;
  int _num_elements;

  // sections might interleave
public:  

  MyStackList_():
    _first(0),
    _num_elements(0) {
    cerr << "MyStackList_::ctor [" << this << "]" << endl;
  }

  MyStackList_(MyStackList_<ValueType>& other) {
    cerr << "MyStackList_::copy ctor [" << this << "]" << endl;
    _copy(other);
  }

  ~MyStackList_() {
    cerr << "MyStackList_::dtor [" << this << "]" << endl;
    clear();
  }

  void push(const ValueType& v) {
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
    cerr << "MyStackList_::print() [" << this << "]" << endl;
    Item* aux=_first;
    int k=0;
    while(aux) {
      cerr << "[" <<k <<"]: " << aux->info << endl;
      ++k;
      aux=aux->next;
    }
  }

  // this enables us to assign stack lists!

                                       
  MyStackList_<ValueType>& operator=(const MyStackList_<ValueType>& other) {
    cerr << "MyStackList_::operator = [" << this << "]" << endl;
    if (&other==this)
      return *this; 
    clear();
    _copy(other);
    return *this; // this enables assigment expressions
  }

  // protected part, all below this label is accessible only to derived classes
protected:
  void _copy(const MyStackList_<ValueType>& other) {
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

using namespace std;

using MyIntStackList    = MyStackList_<int>;
using MyStringStackList = MyStackList_<std::string> ;


int main(int argc, const char** argv) {

  {
    MyStackList_<int> sl1;
    sl1.push(11);
    sl1.push(22);
    sl1.push(33);
    sl1.push(44);
    sl1.print();

    // copy ctor
    MyIntStackList sl2=sl1;
    sl2.push(55);
    sl2.print();
  
    // assignment
    sl2=sl1;
    sl2.print();
  }

  {
    MyStringStackList::ValueType s="pippo";
    cerr << s << endl;
    MyStringStackList sl1;
    sl1.push("aa");
    sl1.push("bb");
    sl1.push("cc");
    sl1.push("dd");
    sl1.print();

    // copy ctor
    MyStringStackList sl2=sl1;
    sl2.push("ee");
    sl2.print();
  
    // assignment
    sl2=sl1;
    sl2.print();
  }

}
