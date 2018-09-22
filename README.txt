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
Napsac: https://www.researchgate.net/profile/Slawomir_Nasuto/publication/221259336_NAPSAC_High_Noise_High_Dimensional_Robust_Estimation_-_it's_in_the_Bag/links/0c96053cd637578f7b000000/NAPSAC-High-Noise-High-Dimensional-Robust-Estimation-its-in-the-Bag.pdf
Prosac: http://cmp.felk.cvut.cz/~matas/papers/chum-prosac-cvpr05.pdf
Evsac: https://ieeexplore.ieee.org/document/6751418/
