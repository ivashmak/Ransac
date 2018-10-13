USAC: A Universal Framework for Random Sample Consensus

Contains LineFitinng tests.
Contains Homograpy Fitting tests.

Avaliable Samplers:
-- Uniform Sampler
-- Napsac Sampler 
-- Evsac Sampler 
-- Prosac Sampler

Install:
mkdir build
cd build/
cmake ..
make -j $(nproc)


Run:
./ransac 


References:
Usac: http://people.inf.ethz.ch/pomarc/pubs/RaguramPAMI13.pdf
Napsac: https://pdfs.semanticscholar.org/cec1/2adbb307124e0c62efbaaa870836c3846b5f.pdf
Prosac: http://cmp.felk.cvut.cz/~matas/papers/chum-prosac-cvpr05.pdf
Evsac: https://ieeexplore.ieee.org/document/6751418/

Local Optimization: http://cmp.felk.cvut.cz/~matas/papers/chum-dagm03.pdf
