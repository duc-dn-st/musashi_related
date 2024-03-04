%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                                 %
% This file is part of HPIPM.                                                                     %
%                                                                                                 %
% HPIPM -- High-Performance Interior Point Method.                                                %
% Copyright (C) 2019 by Gianluca Frison.                                                          %
% Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              %
% All rights reserved.                                                                            %
%                                                                                                 %
% The 2-Clause BSD License                                                                        %
%                                                                                                 %
% Redistribution and use in source and binary forms, with or without                              %
% modification, are permitted provided that the following conditions are met:                     %
%                                                                                                 %
% 1. Redistributions of source code must retain the above copyright notice, this                  %
%    list of conditions and the following disclaimer.                                             %
% 2. Redistributions in binary form must reproduce the above copyright notice,                    %
%    this list of conditions and the following disclaimer in the documentation                    %
%    and/or other materials provided with the distribution.                                       %
%                                                                                                 %
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 %
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   %
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          %
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 %
% ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  %
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    %
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     %
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      %
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   %
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    %
%                                                                                                 %
% Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             %
%                                                                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef hpipm_ocp_qcqp_dim < handle
	
	properties
		C_dim
	end

	methods

		function obj = hpipm_ocp_qcqp_dim(in)
			% create dims struct in C
			if isa(in, 'char')
				% load entire dim from C data file
				file = in;
				compile_mex_one_from_script('ocp_qcqp_dim_load.c', [' -DQP_DATA_H=', file]);
				obj.C_dim = ocp_qcqp_dim_load();
			else
				% create empty dim with horizon N
				N = in;
				obj.C_dim = ocp_qcqp_dim_create(N);
			end
		end

		function set(varargin)
			if nargin==4
				obj = varargin{1};
				field = varargin{2};
				value = varargin{3};
				stage0 = varargin{4};
				ocp_qcqp_dim_set(obj.C_dim, field, value, stage0);
			elseif nargin==5
				obj = varargin{1};
				field = varargin{2};
				value = varargin{3};
				stage0 = varargin{4};
				stage1 = varargin{5};
				ocp_qcqp_dim_set(obj.C_dim, field, value, stage0, stage1);
			else
				disp('hpipm_ocp_qcqp_dim.set: wrong number of input arguments (3 or 4 allowed)');
			end
		end

		function value = get(varargin)
			obj = varargin{1};
			field = varargin{2};
			if strcmp(field, 'N')
				value = ocp_qcqp_dim_get(obj.C_dim, field);
			elseif nargin==3
				stage0 = varargin{3};
				value = ocp_qcqp_dim_get(obj.C_dim, field, stage0);
			else
				disp('hpipm_ocp_qcqp_dim.get: wrong number of input arguments (1 or 2 allowed)');
			end
		end

		function print_C_struct(obj)
			ocp_qcqp_dim_print(obj.C_dim);
		end

		function codegen(obj, file_name, mode)
			ocp_qcqp_dim_codegen(obj.C_dim, file_name, mode);
		end

		function delete(obj)
			%disp('in destructor');
			ocp_qcqp_dim_destroy(obj.C_dim);
		end

	end
end
