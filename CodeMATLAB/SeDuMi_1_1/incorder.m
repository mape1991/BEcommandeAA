%                            [perm, dz] = incorder(At [,Ajc1,ifirst])
% INCORDER
% perm sorts the columns of At greedily, by iteratively picking
%   the 1st unprocessed column with the least number of nonzero
%   subscripts THAT ARE NOT YET COVERED (hence incremental) by
%   the previously processed columns.
% dz has the corresponding incremental sparsity structure, i.e.
%   each column lists only the ADDITIONAL subscripts w.r.t. the
%   previous ones.
%
% ******************** INTERNAL FUNCTION OF SEDUMI ********************
%
% See also getada3, dpr1fact



function [perm, dz] = incorder(At,Ajc1,ifirst)
%
% This file is part of SeDuMi 1.1 by Imre Polik and Oleksandr Romanko
% Copyright (C) 2005 McMaster University, Hamilton, CANADA  (since 1.1)
%
% Copyright (C) 2001 Jos F. Sturm (up to 1.05R5)
%   Dept. Econometrics & O.R., Tilburg University, the Netherlands.
%   Supported by the Netherlands Organization for Scientific Research (NWO).
%
% Affiliation SeDuMi 1.03 and 1.04Beta (2000):
%   Dept. Quantitative Economics, Maastricht University, the Netherlands.
%
% Affiliations up to SeDuMi 1.02 (AUG1998):
%   CRL, McMaster University, Canada.
%   Supported by the Netherlands Organization for Scientific Research (NWO).
%
% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 2 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program; if not, write to the Free Software
% Foundation, Inc.,  51 Franklin Street, Fifth Floor, Boston, MA
% 02110-1301, USA
%

disp('The SeDuMi binaries are not installed.')
disp('In Matlab, launch "install_sedumi" in the folder you put the SeDuMi files.')
disp('For more information see the file Install.txt.')
error(' ')