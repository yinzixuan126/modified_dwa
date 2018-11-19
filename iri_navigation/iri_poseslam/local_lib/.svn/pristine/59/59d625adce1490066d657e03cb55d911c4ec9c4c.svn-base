#ifndef _BTREE_H
#define _BTREE_H
#include <vector>
#include "poseData.h"
#include "poseFilter.h"
#include "flashFilter.h"
#include <algorithm>

struct Data
{
  CPoseData element;
  int left;
  int right;
  uint height;
  int parent;
};

class CBtree
{
  public:
    CBtree();
    CBtree(const CBtree& BT);
    CBtree(const std::vector<CPoseData>& PD);
    ~CBtree();
    
    //gets
    int get_root() const;
    uint get_num_nodes() const;
    std::vector<Data> get_data() const;
    uint get_height() const;
    
    //methods
    void update_tree(const CPoseFilter& PF, const std::vector<CPoseData>& PD);
    void insert_tree(const CPoseData& PDelement);
    std::vector<CPoseData> search_in_tree(const CFlashFilter& FF, const VectorXd& matchArea, const double& t, const int& c) const;
    std::vector<Link > tree_detect_loops(const CFlashFilter& FF, const VectorXd& matchArea, const std::pair<double, double> pdRange) const;
    
    //utilities
    void print_tree() const;
    void print_tree_values() const;

  private:
    int last;
    int root;
    std::vector<int> leaves;
    std::vector<Data> data;
};

#endif
