#include "PostProcessor.hpp"

#include "PostSession.hpp"

void PostProcessor::plug(PostSession *S) { session = S; }

void PostProcessor::begin() {}

void PostProcessor::end() {}

PostProcessor::~PostProcessor() {}
