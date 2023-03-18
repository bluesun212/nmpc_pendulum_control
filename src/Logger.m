classdef Logger < handle
    properties
        % Internal properties
        time_fmt = 'mm:ss.SSS';
        level = 0;
        time;
        eline_length = 0;
    end

    methods(Static)
        % Get the default Logger instance
        function log = get(level)
            persistent def_logger;
            if isempty(def_logger)
                def_logger = Logger(level);
            end
            log = def_logger;
        end
    end

    methods
        % Instantiate a Logger with the specified verbosity level
        function obj = Logger(level)
            if ~nargin
                level = 0;
            end

            obj.level = level;
            obj.time = tic;
        end

        % Resest the logger time
        function reset_timer(obj)
            obj.time = tic;
        end

        % Get the elapsed time since the logger was created or reset
        function t = get_elapsed(obj)
            t = toc(obj.time);
        end

        % Log a message with the specified verbosity and format (fprintf)
        function log(obj, level, fmt, varargin)
            if (obj.level >= level)
                fprintf('%s\n', obj.get_log_string(fmt, varargin{:}));
            end
        end

        % Log/rewrite a status update message for the user
        function log_eline(obj, level, fmt, varargin)
            if (obj.level >= level)
                msg = obj.get_log_string(fmt, varargin{:});
                fprintf([repmat('\b', 1, obj.eline_length) '%s'], msg);
                obj.eline_length = strlength(msg);
            end
        end

        % Stop the status update message and move to the next line
        function stop_eline(obj)
            if (obj.eline_length > 0)
                obj.eline_length = 0;
                fprintf('\n');
            end
        end

        % Internal method - get the text that will be outputted
        function s = get_log_string(obj, fmt, varargin)
            el = obj.get_elapsed();
            el_str = duration(0, 0, el, 'Format', obj.time_fmt);
            msg = sprintf(fmt, varargin{:});
            s = sprintf("%s: %s", el_str, msg);
        end
    end
end