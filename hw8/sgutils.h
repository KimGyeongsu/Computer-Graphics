#ifndef SGUTILS_H
#define SGUTILS_H

#include <vector>

#include "scenegraph.h"

struct RbtNodesScanner : public SgNodeVisitor {
  typedef std::vector<std::shared_ptr<SgRbtNode> > SgRbtNodes;

  SgRbtNodes& nodes_;

  RbtNodesScanner(SgRbtNodes& nodes) : nodes_(nodes) {}

  virtual bool visit(SgTransformNode& node) {
    using namespace std;
    shared_ptr<SgRbtNode> rbtPtr = dynamic_pointer_cast<SgRbtNode>(node.shared_from_this());
    if (rbtPtr)
      nodes_.push_back(rbtPtr);
    return true;
  }
};

inline void dumpSgRbtNodes(std::shared_ptr<SgNode> root, std::vector<std::shared_ptr<SgRbtNode> >& rbtNodes) {
  RbtNodesScanner scanner(rbtNodes);
  root->accept(scanner);
}
/*
[1] space : 
    (1) keyframes�� empty�� �ƴϸ� -> cur_frame data�� scene graph�� copy
[2] 'u' : 
    (1) keyframes�� empty�� �ƴϸ�, scene graph�� cur_Frame data�� copy
    (2) keyframes�� empty��, 'n'�� ���� action ����
[3] 'n'
    (1) keyframe�� empy�� �ƴϸ�, new key frame�� cur_frame �ٷ� �ڿ� ����
    (2) key frame�� empty��, new key frame�� ����.
    ���ϰ��� : scene graph�� new key frame���� �ű�� �̰Ÿ� cur_frame���� ��
[4] '>' : (�����ϸ�) cur_frame�� ���� key frame���� �ű�. �̸� scene�� ǥ��
[5] '<' : (�����ϸ�) cur_frame�� �� key frame���� �ű�. �̸� scene�� ǥ��
[6] 'd' : 
    keyframes�� empty�� �ƴϸ�, cur_frame�� ����
    (1) ���� �� keyframes empty��, cur_frame�� undefined�� ��
    (2) ���� �� empty�� �ƴ϶��, 
        (2-1) �������� first frame�� �ƴϸ�, cur_frame�� �������� �������� ����
        (2-2) �������� first frame�̾��ٸ�, cur_frame�� �������� ���ķ� ����
        �� �� cur_frame�� scene_graph�� �ű�.
[7] : 'i' 
    (1) input key frames from input file(format �˾Ƽ�). cur_frame�� first frame����.
    �̰Ÿ� scene graph�� ����
[8] : 'w'
    (1) output key frames�� output file��. format�� input format�� �����ؾ���.
*/

#endif