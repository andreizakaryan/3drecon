#ifndef UNION_FIND_H
#define UNION_FIND_H

#include <memory>
#include <map>
#include "Point.h"
#include "RGB.h"

// Union-find container.
template <typename Key, typename Value>
class UnionFind
{
    struct Cell
    {
        Cell(const Value& v) :
            value(v) {}
        
        std::shared_ptr<Cell> parent;
        Value value;
    };
    
public:
    UnionFind() :
        invalid(std::make_shared<Cell>(Value()))
    {
    }
    
    // Add a new key with specified value.
    void append(const Key& key, const Value& value)
    {
        cells[key] = std::make_shared<Cell>(value);
    }

    // Get value for equivalency class of key.
    Value at(const Key& key) const
    {
        std::shared_ptr<Cell> c = find(key);
        if (c)
            return c->value;
        return Value();
    }

    // Set value for equivalency class of key.
    void set(const Key& key, const Value& value)
    {
        std::shared_ptr<Cell> c = find(key);
        if (c)
            c->value = value;
    }
    
    // Merge equivalency classes of k1 and k2.
    void merge(const Key& k1, const Key& k2)
    {
        std::shared_ptr<Cell> root1 = find(k1);
        std::shared_ptr<Cell> root2 = find(k2);
        
        if (root1 != root2)
            root1->parent = root2;
    }

    // Destroy equivalency class of key and reset each item to its initial value.
    void reset(const Key& key)
    {
        std::shared_ptr<Cell> c = find(key);
        if (c)
            c->parent = invalid;
    }
    
private:
    std::shared_ptr<Cell> find(const Key& key) const
    {
        auto found = cells.find(key);
        if (found == cells.end())
            return std::shared_ptr<Cell>();
        std::shared_ptr<Cell> c = found->second;
        
        std::shared_ptr<Cell> root = c;
        while (root->parent)
            root = root->parent;

        while (c != root)
        {
            std::shared_ptr<Cell> p = c->parent;
            c->parent = root;
            c = p;
        }

        if (root == invalid)
        {
            root = std::make_shared<Cell>(found->second->value);
            found->second = root;
        }
        
        return root;
    }
    
    mutable std::map<Key, std::shared_ptr<Cell>> cells;
    std::shared_ptr<Cell> invalid;
};

typedef UnionFind<SharedPoint, std::pair<RGB, bool>> UnionFindPlanes;

#endif // UNION_FIND_H

