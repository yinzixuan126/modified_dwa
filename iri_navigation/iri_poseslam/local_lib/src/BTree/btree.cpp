#include "btree.h"

// Balanced tree constructor.
// Builds a new balanced tree of poses.
// The balanced tree consructor can be used with the following parameters
//   - Another balanced tree. Copy constructor.
//   - A data set in the form of a cell array (possibly empty).

CBtree::CBtree()
:
  last(-1),
  root(-1),
  leaves(1000, -1)
{
  data.reserve(1000);
}

CBtree::CBtree(const std::vector<CPoseData>& PD)
  :
  last(-1),
  root(-1),
  leaves(1000, -1)
{
  uint nd = PD.size();
  uint n = std::max(nd, uint(1000));
  data.reserve(n);
  for (uint i=0; i<nd; i++) 
    insert_tree(PD[i]);
}

CBtree::CBtree(const CBtree& BT)
  :
  last(BT.last),
  root(BT.root),
  leaves(BT.leaves),
  data(BT.data)
{
}

CBtree::~CBtree()
{
}

// GETS

int CBtree::get_root() const
{
  return root;
}

uint CBtree::get_num_nodes() const
{
  return uint(last + 1);
}

std::vector<Data> CBtree::get_data() const
{
  return data;
}

uint CBtree::get_height() const
{
  return data[root].height;
}

// METHODS

void CBtree::update_tree(const CPoseFilter& PF, const std::vector<CPoseData>& PD)
{
  // Updates the information in a BTree: the information stored in the leaves and the internal nodes of a BTree.
  // This function is used after loop closures, when the information for all poses are likely to signifiantly change and it is much faster that re-building the tree from scratch.
  // Parameters:
  //   - PF The generic pose filter (indexes relationships...)
  //   - PD The set with the PoseData (mean, covariance and cross covariance with the current pose) for each pose.
  
  int c = root;
  int s = 0;
  bool done = (c == 0);
  int p;
  
  while (!done)
  {
    // for leafs there is no need to go to lower levels
    if (data[c].height == 1)
      s=2;
    
    switch (s)
    {
      case 0:
        // right tree is still to be udpated -> proceed
        c = data[c].right;
        s = 0;
        break;
      case 1:
        // left tree is still to be updated -> proceed
        c = data[c].left;
        s = 0;
        break;
      case 2:
        // right and left trees are updated -> update the current node and go up
        if (data[c].height == 1)
          data[c].element = PD[PF.step_2_state(data[c].element.get_id())];
        else
        {
          int l = data[c].left;
          int r = data[c].right;
          data[c].element = data[c].element.pd_union(data[l].element, data[r].element);
        }

        // The current sub-tree is compleated, proceed with the upper levels.
        // if the current sub-tree is the right branch of the upper level s is set to 1 to proceed with the left branch.
        // if the current sub-tree is the left branch, we can proceed to update the parent node (we move to the parent node with s=2)
        
        p = data[c].parent;
        if (p == -1)
          done = true;
        else
        {
          if (data[p].right == c)
            s = 1;
          else 
            s = 2;
          c = p;
        }
    }
  }
}

void CBtree::insert_tree(const CPoseData& element)
{
  //  Inserts an element in a BTree. Adds an element to the rigth-most extrem of a BTree and then it rebalances the tree. 
  //  Moreover, the information of the tree nodes from the new leaf to the root are updated with the information of the new element.

  if (last==-1) // is the first element added
  {
    last=0;
    root=0;
    
    leaves[element.get_id()] = last;
    
    Data data_0;
    data_0.element = element;
    data_0.left = -1;
    data_0.right = -1;
    data_0.height = 1;
    data_0.parent = -1;
    
    data.push_back(data_0);
  }
  else
  {
    int p = data[last].parent; // old parent
    int new_parent = last + 1; // n2 // place for the root of the last element up to now and the element we are adding to the tree now            
    int new_element = last + 2; // n1 // Place for element we are adding to the tree right now
    
    // reserve space in vectors if needed
    if (leaves.size() <= element.get_id()){ 
      leaves.insert(leaves.end(), 1000, -1);
    }
    if (int(data.capacity()) <= last + 3)
    {
      data.reserve(data.size() + size_t(1000));
    }
    
    // Element we are adding to the tree (n1 in Matlab)
    leaves[element.get_id()] = new_element;
    
    Data data_new_element;
    data_new_element.element = element;
    data_new_element.left = -1;
    data_new_element.right = -1;
    data_new_element.height = 1;
    data_new_element.parent = new_parent;

    //Root of the last element up to now and the element we are adding to the tree now (n2 in Matlab)
    data[last].parent = new_parent;
    
    Data data_new_parent;
    data_new_parent.element = data_new_parent.element.pd_union(element, data[last].element); 
    data_new_parent.left = last;
    data_new_parent.right = new_element;
    data_new_parent.height = 2;
    data_new_parent.parent = p;
    
    // if p (old parent) exist, the new parent is his right
    if (p >= 0) data[p].right = new_parent;
    
    // if 'last' was root, the new_parent is the new root
    if (root == last) root = new_parent;
    
    // Store both new elements
    data.push_back(data_new_parent);
    data.push_back(data_new_element);
    
    last = new_element;
    
    // now navigate back to the root, updating the internal tree nodes and re-balancing the tree
    while (p != -1)
    {
      int l = data[p].left;
      int r = data[p].right;
      
      uint hl = data[l].height;
      uint hr = data[r].height;
      
      if (hr-hl > 1)
      {
        // Re-balance current node
        int rr = data[r].right;
        int rl = data[r].left;
        
        data[p].left = r;
        data[p].right = rr;
        
        data[r].left = l;
        data[r].right = rl;
        
        data[rr].parent = p;
        data[l].parent = r;
        
        data[r].element = data[r].element.pd_union(data[l].element, data[rl].element); 
        data[r].height = std::max(data[l].height, data[rl].height) + 1;
      }
      
      l = data[p].left;
      r = data[p].right;
       
      data[p].element = data[p].element.pd_union(data[l].element, data[r].element); 
      data[p].height = std::max(data[l].height, data[r].height) + 1;

      // proceed up to the root
      p = data[p].parent;
    }
  }
}

std::vector<CPoseData> CBtree::search_in_tree(const CFlashFilter& FF, const VectorXd& matchArea, const double& t, const int& c) const
{
  // the output is just a set with the elements matching the search critera. (recursive function)
  // Input parameters
  //   - FF The Filter to use in the search
  //   - matchArea The area around the previous pose where the new pose should be to be considered a neighbour (this is vector 'v' in the papers).
  //   - t The probability threshold (this is 's' in the papers).
  //   - c The BTree node from where to start the search. This must be the root node for the initial call.

  std::pair<double, double> p = FF.prob_distance_2_posedata_above_threshold(data.at(c).element, matchArea, t);

  double pMin = p.first;
  double pMax = p.second;
  
  if (pMax < t)
  {
    std::vector<CPoseData> neighbours;  
    neighbours.clear(); // empty neighbours
    return neighbours;
  }
  else
  {
    if (pMin > t)
    {
      std::vector<uint> ids = data.at(c).element.get_interval_id();
      uint nIds = ids.size();
      std::vector<CPoseData> neighbours(nIds);

      for (uint i = 0; i < nIds; i++)
        neighbours.at(i) = data.at(leaves.at(ids.at(i))).element;
      
      return neighbours;
    }
    else
    {
      if (data.at(c).height > 1)
      {
        std::vector<CPoseData> neighboursL = search_in_tree(FF, matchArea, t, data.at(c).left);
        std::vector<CPoseData> neighboursR = search_in_tree(FF, matchArea, t, data.at(c).right);
        std::vector<CPoseData> neighbours;
        neighbours.reserve(neighboursL.size() + neighboursR.size());
        neighbours.assign(neighboursL.begin(), neighboursL.end());
        neighbours.insert(neighbours.end(), neighboursR.begin(), neighboursR.end());
        
        return neighbours;
      }
      else
      {
        std::vector<CPoseData> neighbours(1);
        neighbours.at(0) = data.at(c).element;
        return neighbours;
      }
    } 
  }
}

std::vector<Link> CBtree::tree_detect_loops(const CFlashFilter& FF, const VectorXd& matchArea, const std::pair<double, double> pdRange) const
{
  // Propose loops based on the distance between poses using a tree for the search. 
  // Parameters
  //  - matchArea: Area around the current robot's pose where to search for neighbouring poses. This is a pose displacement expressed in the frame of the current robot's pose.
  //  - pdRange: Interval [p1,p2]. If a pose has probabiliby higher than p1 of being in the matchArea, it is consired a neighbouring pose for the current robot's pose. 
  //                               If the probability is above p2, the it is considered a very close neighbouring pose.
  //
  // Output: A vector with the neighbours where each element is a pair containing:
  //  - step: The identifier of the neighbour pose
  //  - pd: The neighbouring probability (always larger than pdRange[1]).
  
  const int r = get_root();

  // Empty Tree --> Empty candidates_list
  if (r == -1)
  {
    std::vector<Link> candidates_list(0);
    return candidates_list;
  }
  // Not empty candidates_list
  else
  {
    std::vector<CPoseData> neighbours = search_in_tree(FF, matchArea, pdRange.first, r);
    std::vector<Link> candidates_list(neighbours.size());
    for (uint i = 0; i < neighbours.size(); i++)
    {
      candidates_list.at(i).step = neighbours[i].get_id();
      candidates_list.at(i).pd = FF.prob_distance_2_pose_below_threshold(matchArea, FF.get_relative_displacement_2_pose(neighbours[i].get_id()));
    }
    return candidates_list;
  }
}

// utilities
void CBtree::print_tree() const
{
  int i = root;
  uint total_height = data[root].height;
  bool done[last+1];
  for (int j=0; j<=last; j++) done[j]=false;
    bool finished = false;
  
  printf("Printing BTree... \n\n");
  
  while (!finished)
  { 
    int right = data[i].right;
    int left = data[i].left;
      
    if (!done[i]) // isn't printed yet
    {
      // print and mark as done
      printf("%i", i);
      if (right == -1) printf(" - step: %i",data[i].element.get_id());
      done[i] = true;
    }
    else if (right != -1) //it's printed, if has sons..
    {  
      if (!done[right]) //if the right one isn't printed
      {
        i = right; // go down and print tabulator
        printf("\t");
      }
      else if (!done[left]) //if the left one isn't printed
      {
        i = left; // go down and print enter and tabulators
        printf("\n");
        for (uint j = 0; j < total_height - data[i].height; j++) printf("\t");
      }
      else //it has sons but they are already printed
        i = data[i].parent;
    }  
    else //it's printed and it hasn't any sons
      i = data[i].parent;
    
    finished = true;
    for (int j = 0; j <= last; j++) 
      if (done[j] == false)
        finished = false;
    if (finished) 
      printf("\n\nend of printing BTree\n\n");
  }  
}

void CBtree::print_tree_values() const
{
  int i = root;
  bool done[last+1];
  for (int j=0; j<=last; j++) done[j]=false;
  bool finished = false;
  
  printf("Printing BTree values... \n\n");
  printf("root : %i\n", root);
  printf("last : %i\n", last);
  
  while (!finished)
  { 
    int right = data[i].right;
    int left = data[i].left;
      
    if (!done[i]) // isn't printed yet
    {
      // print and mark as done
      printf("INDEX %i\n", i);
      data[i].element.print_pd();
      done[i] = true;
    }
    else if (right != -1) //it's printed, if has sons..
    {  
      if (!done[right]) //if the right one isn't printed, go down
        i = right;
      else if (!done[left]) //if the left one isn't printed, go down
        i = left;
      else //it has sons but they are already printed, go up
        i = data[i].parent;
    }  
    else //it's printed and it hasn't any sons, go up
      i = data[i].parent;
    
    finished = true;
    for (int j = 0; j <= last; j++) 
      if (done[j] == false)
        finished = false;
    if (finished) 
      printf("end of printing BTree values\n\n");
  }  
}
