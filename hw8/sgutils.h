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
    (1) keyframes가 empty가 아니면 -> cur_frame data를 scene graph로 copy
[2] 'u' : 
    (1) keyframes가 empty가 아니면, scene graph를 cur_Frame data로 copy
    (2) keyframes가 empty면, 'n'에 따른 action 진행
[3] 'n'
    (1) keyframe이 empy가 아니면, new key frame을 cur_frame 바로 뒤에 생성
    (2) key frame이 empty면, new key frame을 만듦.
    동일과정 : scene graph를 new key frame으로 옮기고 이거를 cur_frame으로 함
[4] '>' : (가능하면) cur_frame을 다음 key frame으로 옮김. 이를 scene에 표시
[5] '<' : (가능하면) cur_frame을 전 key frame으로 옮김. 이를 scene에 표시
[6] 'd' : 
    keyframes가 empty가 아니면, cur_frame을 지움
    (1) 지운 후 keyframes empty면, cur_frame을 undefined로 함
    (2) 지운 후 empty가 아니라면, 
        (2-1) 지워진게 first frame이 아니면, cur_frame을 지워진거 직전으로 설정
        (2-2) 지워진게 first frame이었다면, cur_frame을 지워진거 직후로 설정
        그 후 cur_frame을 scene_graph로 옮김.
[7] : 'i' 
    (1) input key frames from input file(format 알아서). cur_frame은 first frame으로.
    이거를 scene graph로 복사
[8] : 'w'
    (1) output key frames를 output file로. format은 input format과 동일해야함.
*/

#endif